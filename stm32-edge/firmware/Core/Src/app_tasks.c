/**
  ******************************************************************************
  * @file    app_tasks.c
  * @brief   W2 4태스크 골격 + 마감시한 이벤트→액션 지연 측정 + IMU 100Hz 태스크
  *
  * 측정 규칙(CLAUDE.md): 반복 >= 100, 워밍업 제외, 중앙값/p99/max 병기.
  * 메모리 규칙: 동적 할당 금지 - 태스크/큐/버퍼 전부 정적.
  ******************************************************************************
  */
#include "app_tasks.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "main.h"
#include "i2c.h"
#include "dwt.h"
#include "mpu6050.h"

#include <stdio.h>

/* ------------------------------------------------------------------ 파라미터 */

#define PRIO_ACTION        48u
#define PRIO_SENSOR        40u
#define PRIO_LOG           16u
#define PRIO_INFER          8u

#define SENSOR_PERIOD_MS   10u   /* 100Hz */
#define EVENT_PERIOD_MS    50u   /* 마감시한 이벤트 20Hz */
#define DEADLINE_MS         1u   /* 잠정 마감시한 - 초과 횟수만 센다 */

#define WARMUP_N           16u   /* 태스크 기동 후 첫 N샘플은 통계 제외 */
#define ACT_BATCH_N       128u   /* 액션 지연: 배치당 샘플 수 (6.4초/배치) */
#define SEN_BATCH_N       256u   /* 센서 주기: 배치당 샘플 수 (2.56초/배치) */

#define INFER_BUSY_MS      20u   /* 추론 스텁: 20ms 연산 + 30ms 휴지 = 40% 부하 */
#define INFER_IDLE_MS      30u

#define WIN_N             256u   /* W3 추론 입력용 가속도 링버퍼 (2^n 필수) */

/* ------------------------------------------------------------------ 로그 큐 */

typedef enum
{
  APP_MSG_ACT = 0,   /* 액션 지연 배치 */
  APP_MSG_SEN,       /* 센서 주기 배치 */
  APP_MSG_ERR,       /* 오류 통지 (aux0 = 코드) */
} app_msg_type_t;

typedef struct
{
  uint8_t   type;      /* app_msg_type_t */
  uint8_t   _pad;
  uint16_t  n;
  uint32_t  batch;
  uint32_t *buf;       /* 원본 사이클 버퍼 - 로그 태스크가 제자리 정렬(파괴) */
  volatile uint8_t *owner;  /* 버퍼 소유권 플래그. 로그가 다 쓰면 0으로 되돌린다 */
  uint32_t  aux0;      /* ACT: 마감 초과 수 / SEN: 읽기 실패 수 / ERR: 코드 */
  uint32_t  aux1;      /* ACT: 이벤트 유실 수(coalesced) */
  uint32_t  aux2;      /* 로그 백프레셔로 측정에서 빠진 샘플 수 */
  int16_t   ax, ay, az;/* SEN: 생존 신호용 최근 샘플 */
} app_msg_t;

#define ERR_TIM2_INIT  1u

#define LOGQ_LEN 8u
static StaticQueue_t  s_logq_cb;
static uint8_t        s_logq_storage[LOGQ_LEN * sizeof(app_msg_t)];
static QueueHandle_t  s_logq;

/* ------------------------------------------------------------------ 정적 태스크 */

#define STK_ACTION 256u
#define STK_SENSOR 384u
#define STK_LOG    512u   /* printf가 여기서만 돈다 */
#define STK_INFER  192u

static StaticTask_t s_tcb_action, s_tcb_sensor, s_tcb_log, s_tcb_infer;
static StackType_t  s_stk_action[STK_ACTION];
static StackType_t  s_stk_sensor[STK_SENSOR];
static StackType_t  s_stk_log[STK_LOG];
static StackType_t  s_stk_infer[STK_INFER];

static TaskHandle_t s_action_task;

/* ------------------------------------------------------------------ 측정 버퍼 */

/* 더블 버퍼: 한쪽을 로그 태스크가 정렬하는 동안 반대쪽에 계속 쌓는다.
 * busy 플래그가 소유권을 표시한다 - 이게 없으면 로그가 밀렸을 때 생산자가
 * 두 바퀴 돌아 "아직 정렬 중인 버퍼"를 덮어쓴다. */
static uint32_t s_act_lat[2][ACT_BATCH_N];
static uint32_t s_sen_int[2][SEN_BATCH_N];
static volatile uint8_t s_act_busy[2];
static volatile uint8_t s_sen_busy[2];

/* W3 대비: 추론 입력 윈도우 링버퍼 (지금은 쌓기만 한다) */
static mpu6050_accel_t s_win[WIN_N];
static volatile uint32_t s_win_wr;

/* ------------------------------------------------------------------ TIM2 이벤트 */

static TIM_HandleTypeDef s_htim2;
static volatile uint32_t s_event_t0;      /* ISR이 찍는 이벤트 발생 시점(사이클) */
static uint32_t          s_deadline_cyc;  /* DEADLINE_MS의 사이클 환산 */

/**
  * TIM2를 1MHz 틱으로 맞춰 EVENT_PERIOD_MS 주기 업데이트 인터럽트를 건다.
  * NVIC 우선순위 6 = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)보다 낮게
  * (숫자 크게) 잡아야 ISR에서 FromISR API를 부를 수 있다.
  */
static bool prv_tim2_start(void)
{
  uint32_t pclk1;
  uint32_t tclk;

  __HAL_RCC_TIM2_CLK_ENABLE();

  /* RM0383: APB 프리스케일러가 1이 아니면 타이머 클럭 = PCLK x2 */
  pclk1 = HAL_RCC_GetPCLK1Freq();
  tclk  = (HAL_RCC_GetHCLKFreq() == pclk1) ? pclk1 : (pclk1 * 2u);

  s_htim2.Instance               = TIM2;
  s_htim2.Init.Prescaler         = (tclk / 1000000u) - 1u;          /* 1us 틱 */
  s_htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  s_htim2.Init.Period            = (EVENT_PERIOD_MS * 1000u) - 1u;
  s_htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  s_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&s_htim2) != HAL_OK)
  {
    return false;
  }

  /* Init이 UG로 만든 가짜 UIF를 지운다 - 안 지우면 시작 즉시 이벤트 1발 */
  __HAL_TIM_CLEAR_FLAG(&s_htim2, TIM_FLAG_UPDATE);

  HAL_NVIC_SetPriority(TIM2_IRQn, 6u, 0u);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  return (HAL_TIM_Base_Start_IT(&s_htim2) == HAL_OK);
}

/**
  * 마감시한 이벤트 ISR. HAL_TIM_IRQHandler를 거치지 않는다 - 타임스탬프와
  * 노티피케이션만 하는 최소 경로여야 측정에 ISR 오버헤드가 안 섞인다.
  */
void TIM2_IRQHandler(void)
{
  if ((TIM2->SR & TIM_SR_UIF) != 0u)
  {
    BaseType_t hpw = pdFALSE;

    TIM2->SR = ~TIM_SR_UIF;        /* rc_w0: 0을 쓴 비트만 지워진다 */
    s_event_t0 = dwt_now();

    vTaskNotifyGiveFromISR(s_action_task, &hpw);
    portYIELD_FROM_ISR(hpw);
  }
}

/* ------------------------------------------------------------------ 액션 태스크 */

/**
  * 하드 마감시한 담당. 이벤트 노티피케이션에 깨어나 지연을 기록하고
  * 로컬 액션(LD2 토글)을 수행한다. 일부러 이 이상 아무것도 안 한다 -
  * 통계는 로그 태스크 몫.
  */
static void prv_action_task(void *arg)
{
  uint32_t warm = 0u, idx = 0u, batch = 0u;
  uint32_t miss = 0u, lost = 0u, skip = 0u;
  uint8_t  side = 0u;

  (void)arg;

  if (!prv_tim2_start())
  {
    app_msg_t m = { .type = APP_MSG_ERR, .aux0 = ERR_TIM2_INIT };
    (void)xQueueSend(s_logq, &m, portMAX_DELAY);
    vTaskDelete(NULL);
  }

  for (;;)
  {
    uint32_t nvals = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint32_t lat   = dwt_now() - s_event_t0;

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);   /* <- 로컬 액션 */

    if (nvals > 1u)
    {
      lost += nvals - 1u;   /* 못 따라간 이벤트 - t0가 덮여서 지연도 최신 것 기준 */
    }
    if (warm < WARMUP_N)
    {
      warm++;
      continue;
    }
    if (lat > s_deadline_cyc)
    {
      miss++;
    }

    if (s_act_busy[side] != 0u)
    {
      skip++;               /* 로그가 아직 이 버퍼를 들고 있다 - 이번 샘플만 버린다 */
      continue;             /* (액션 자체는 이미 수행했다. 측정만 빠진다) */
    }

    s_act_lat[side][idx++] = lat;
    if (idx >= ACT_BATCH_N)
    {
      app_msg_t m = {
        .type = APP_MSG_ACT, .n = ACT_BATCH_N, .batch = batch,
        .buf = s_act_lat[side], .owner = &s_act_busy[side],
        .aux0 = miss, .aux1 = lost, .aux2 = skip,
      };
      s_act_busy[side] = 1u;              /* 소유권을 로그 태스크로 넘긴다 */
      if (xQueueSend(s_logq, &m, 0u) != pdTRUE)
      {
        s_act_busy[side] = 0u;            /* 큐 포화 - 배치 드롭하고 소유권 회수 */
      }
      side ^= 1u;
      idx = 0u; miss = 0u; lost = 0u; skip = 0u;
      batch++;
    }
  }
}

/* ------------------------------------------------------------------ 센서 태스크 */

/**
  * MPU6050을 vTaskDelayUntil로 100Hz 폴링. 실제 주기를 DWT로 재서
  * "태스크 환경에서도 100Hz가 유지되는가"를 배치 통계로 보고한다.
  */
static void prv_sensor_task(void *arg)
{
  TickType_t last;
  uint32_t prev, warm = 0u, idx = 0u, batch = 0u, fails = 0u, skip = 0u;
  uint8_t  side = 0u;
  mpu6050_accel_t a = { 0 };

  (void)arg;

  /* 브링업이 이미 깨웠어도 무해(멱등). 브링업을 건너뛴 부팅도 여기서 복구 */
  (void)mpu6050_wake(&hi2c1);

  last = xTaskGetTickCount();
  prev = dwt_now();

  for (;;)
  {
    uint32_t t0;

    vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSOR_PERIOD_MS));

    t0 = dwt_now();

    if (mpu6050_read_accel(&hi2c1, &a))
    {
      s_win[s_win_wr & (WIN_N - 1u)] = a;   /* W3 추론 입력 윈도우 */
      s_win_wr++;
    }
    else
    {
      fails++;
    }

    if (warm < WARMUP_N)
    {
      warm++;
      prev = t0;   /* 워밍업 동안은 구간도 버린다 */
      continue;
    }

    if (s_sen_busy[side] != 0u)
    {
      skip++;               /* 로그가 아직 이 버퍼를 들고 있다 */
      prev = t0;            /* prev는 갱신해야 다음 구간이 이어진다 */
      continue;
    }

    s_sen_int[side][idx++] = t0 - prev;
    prev = t0;

    if (idx >= SEN_BATCH_N)
    {
      app_msg_t m = {
        .type = APP_MSG_SEN, .n = SEN_BATCH_N, .batch = batch,
        .buf = s_sen_int[side], .owner = &s_sen_busy[side],
        .aux0 = fails, .aux2 = skip,
        .ax = a.x, .ay = a.y, .az = a.z,
      };
      s_sen_busy[side] = 1u;
      if (xQueueSend(s_logq, &m, 0u) != pdTRUE)
      {
        s_sen_busy[side] = 0u;
      }
      side ^= 1u;
      idx = 0u; fails = 0u; skip = 0u;
      batch++;
    }
  }
}

/* ------------------------------------------------------------------ 추론 스텁 */

/**
  * W2에서는 진짜 모델 대신 "CPU를 태우는 무거운 payload"만 흉내낸다.
  * 20ms 연산 + 30ms 휴지 = 40% 부하. 이 부하가 도는 와중에도 액션 지연이
  * 낮게 유지되는지가 W2 베이스라인이고, W3에서 이걸 monolithic/청킹으로 바꾼다.
  */
static void prv_infer_task(void *arg)
{
  volatile uint32_t acc = 1u;
  const uint32_t busy_cyc = (SystemCoreClock / 1000u) * INFER_BUSY_MS;

  (void)arg;

  for (;;)
  {
    uint32_t t0 = dwt_now();
    while (dwt_elapsed(t0) < busy_cyc)
    {
      acc = acc * 1664525u + 1013904223u;   /* 최적화로 안 사라지는 더미 정수 연산 */
    }
    vTaskDelay(pdMS_TO_TICKS(INFER_IDLE_MS));
  }
}

/* ------------------------------------------------------------------ 로그 태스크 */

static uint32_t prv_cyc_to_us10(uint32_t cyc)   /* 사이클 -> 0.1us 단위 */
{
  return cyc * 10u / (SystemCoreClock / 1000000u);
}

static void prv_log_task(void *arg)
{
  app_msg_t   m;
  dwt_stats_t st;

  (void)arg;

  printf("\r\n=== W2 tasks up: action(%u) sensor(%u) log(%u) infer(%u) ===\r\n",
         (unsigned)PRIO_ACTION, (unsigned)PRIO_SENSOR,
         (unsigned)PRIO_LOG, (unsigned)PRIO_INFER);
  printf("event=%lums deadline=%lums batch: act=%lu sen=%lu warmup=%lu\r\n",
         (unsigned long)EVENT_PERIOD_MS, (unsigned long)DEADLINE_MS,
         (unsigned long)ACT_BATCH_N, (unsigned long)SEN_BATCH_N,
         (unsigned long)WARMUP_N);
  dwt_csv_header();

  for (;;)
  {
    if (xQueueReceive(s_logq, &m, portMAX_DELAY) != pdTRUE)
    {
      continue;
    }

    if (m.type == APP_MSG_ERR)
    {
      printf("[ERR] code=%lu (1=TIM2 init fail)\r\n", (unsigned long)m.aux0);
      continue;
    }

    dwt_stats(m.buf, m.n, &st);

    if (m.type == APP_MSG_ACT)
    {
      uint32_t med10 = prv_cyc_to_us10(st.med);
      uint32_t p9910 = prv_cyc_to_us10(st.p99);
      uint32_t max10 = prv_cyc_to_us10(st.max);
      printf("[ACT] batch=%lu n=%u lat_us med=%lu.%lu p99=%lu.%lu max=%lu.%lu"
             " miss(%lums)=%lu lost=%lu skip=%lu\r\n",
             (unsigned long)m.batch, (unsigned)m.n,
             (unsigned long)(med10 / 10u), (unsigned long)(med10 % 10u),
             (unsigned long)(p9910 / 10u), (unsigned long)(p9910 % 10u),
             (unsigned long)(max10 / 10u), (unsigned long)(max10 % 10u),
             (unsigned long)DEADLINE_MS, (unsigned long)m.aux0,
             (unsigned long)m.aux1, (unsigned long)m.aux2);
      dwt_csv_row("action_latency", &st);
    }
    else
    {
      uint32_t med10 = prv_cyc_to_us10(st.med) / 10u;   /* 주기는 us 정수면 충분 */
      printf("[SEN] batch=%lu n=%u interval_us med=%lu p99=%lu max=%lu"
             " fail=%lu skip=%lu accel=(%d,%d,%d)\r\n",
             (unsigned long)m.batch, (unsigned)m.n,
             (unsigned long)med10,
             (unsigned long)(prv_cyc_to_us10(st.p99) / 10u),
             (unsigned long)(prv_cyc_to_us10(st.max) / 10u),
             (unsigned long)m.aux0, (unsigned long)m.aux2,
             (int)m.ax, (int)m.ay, (int)m.az);
      dwt_csv_row("imu_interval_task", &st);
    }

    if (m.owner != NULL)
    {
      *m.owner = 0u;   /* 버퍼 소유권 반환 - 이제 생산자가 다시 써도 된다 */
    }
  }
}

/* ------------------------------------------------------------------ 초기화 */

void app_tasks_init(void)
{
  s_deadline_cyc = (SystemCoreClock / 1000u) * DEADLINE_MS;

  s_logq = xQueueCreateStatic(LOGQ_LEN, sizeof(app_msg_t),
                              s_logq_storage, &s_logq_cb);
  configASSERT(s_logq != NULL);

  (void)xTaskCreateStatic(prv_log_task, "log", STK_LOG, NULL,
                          PRIO_LOG, s_stk_log, &s_tcb_log);
  (void)xTaskCreateStatic(prv_infer_task, "infer", STK_INFER, NULL,
                          PRIO_INFER, s_stk_infer, &s_tcb_infer);
  (void)xTaskCreateStatic(prv_sensor_task, "sensor", STK_SENSOR, NULL,
                          PRIO_SENSOR, s_stk_sensor, &s_tcb_sensor);
  s_action_task = xTaskCreateStatic(prv_action_task, "action", STK_ACTION, NULL,
                                    PRIO_ACTION, s_stk_action, &s_tcb_action);
  configASSERT(s_action_task != NULL);
}
