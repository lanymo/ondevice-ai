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

/* 합성 부하 태스크의 두 우선순위. LOW는 액션보다 아래(정상 배치),
 * HIGH는 액션보다 위(일부러 뒤집은 배치). configMAX_PRIORITIES=56이라 50은 유효. */
#define PRIO_LOAD_LOW       8u
#define PRIO_LOAD_HIGH     50u

#define SENSOR_PERIOD_MS   10u   /* 100Hz */
#define EVENT_PERIOD_MS    50u   /* 마감시한 이벤트 20Hz */

/* 관측 기준선 - "합격선"이 아니라 초과 횟수를 세는 눈금이다.
 * 이 데모의 로컬 액션(LD2)에는 물리적 마감시한이 없다(docs/system-design.md 1-2).
 * 산출물은 이 값의 준수 여부가 아니라 config별 지연 분포의 이동이다.
 * 1ms를 고른 이유: 정상 배치 실측 9.2us의 100배 위 = 정상 config에서는 절대
 * 안 걸리고, 청킹 크기를 1ms 근처로 쓸어내릴 때 비로소 반응하는 눈금이라서. */
#define DEADLINE_MS         1u

#define WARMUP_N           16u   /* 태스크 기동/config 전환 후 첫 N샘플은 통계 제외 */
#define ACT_BATCH_N       128u   /* 액션 지연: 배치당 샘플 수 (6.4초/배치) */
#define SEN_BATCH_N       256u   /* 센서 주기: 배치당 샘플 수 (2.56초/배치) */
#define INF_BATCH_N       128u   /* 부하 잡 소요시간: 배치당 샘플 수 (6.8초/배치) */

/* ---- 합성 부하 (W3 축 A) ----------------------------------------------
 * 이건 추론이 아니다. 진짜 추론은 실측상 CPU의 0.2%라(system-design.md 1-3)
 * 스케줄링 실험의 독립변수가 될 수 없어서, 부하를 별도 태스크로 분리했다.
 * 추론 사이클은 따로 정직하게 잰다 (measurements/inference_cycles.csv).
 *
 * 주기 53ms는 이벤트 주기 50ms와 서로소로 고른 값이다. 두 주기가 같거나
 * 약수 관계면 이벤트가 부하 구간에 걸리는 위상이 고정돼, 결과가 "얼마나 오래
 * 켜뒀나"에 좌우된다. 서로소면 위상이 50x53ms=2.65초마다 한 바퀴 돌아
 * 한 배치(6.4초) 안에서 위상 공간이 2.4바퀴 쓸린다. */
#define LOAD_BUSY_MS       20u   /* 잡 1회의 순수 연산량 (청크 수와 무관하게 고정) */
#define LOAD_PERIOD_MS     53u   /* 잡 주기. 부하율 = 20/53 = 37.7% (전 config 동일) */

/* W3 추론 입력 윈도우. model/config.py의 WIN_LEN=128 / HOP=64와 일치시킬 것 -
 * 어긋나면 학습된 특징 분포와 온보드 특징이 다른 것을 재게 된다. (2^n 필수) */
#define WIN_N             128u
#define WIN_HOP            64u

/* ------------------------------------------------------------------ 스윕 config */

/**
  * 축 A 스케줄링 스윕. 한 바이너리가 config를 순회한다 - config마다 따로
  * 빌드하면 "컴파일이 달라서 그런 것 아니냐"는 교란이 남는다.
  *
  * 조작 대상은 두 개뿐이다: (1) 부하 태스크의 우선순위 배치, (2) 선점 지점.
  * 부하의 "양"은 독립변수가 아니다 - 부하 37.7%를 걸어도 최상위 태스크의
  * 지연은 33,024샘플에서 폭 167ns였다(2026-08-05 실측). 고정우선순위 선점
  * 스케줄러에서 낮은 우선순위의 부하는 원리적으로 위 태스크에 닿지 않는다.
  *
  * 청킹의 경계가 taskYIELD()가 아니라 vTaskDelay()인 이유:
  * taskYIELD()는 스케줄러를 다시 돌릴 뿐이고, 그러면 "준비된 태스크 중
  * 최고 우선순위"가 다시 선택된다 - 즉 부하 태스크 자신이다. 자기보다
  * 낮은 우선순위에게 CPU를 넘기려면 실제로 블록해야 한다.
  */
typedef struct
{
  const char *name;
  uint8_t     load_on;    /* 0 = 부하 없음 (대조군) */
  uint8_t     prio;       /* 부하 태스크 우선순위 */
  uint16_t    chunks;     /* 1 = monolithic. N = LOAD_BUSY_MS/N씩 쪼개고 사이에 블록 */
} sweep_cfg_t;

static const sweep_cfg_t s_cfg[] = {
  /* name          load prio            chunks  기대 (액션 지연 max) */
  { "L0-noload",     0u, PRIO_LOAD_LOW,     1u },  /* 대조군: 부하 자체가 없음 */
  { "A-mono",        1u, PRIO_LOAD_LOW,     1u },  /* 정상 배치 + 37.7% 부하 */
  { "B-mono",        1u, PRIO_LOAD_HIGH,    1u },  /* 우선순위 뒤집기 -> 20ms 대기 */
  { "C-chunk5ms",    1u, PRIO_LOAD_HIGH,    4u },  /* 5ms 청크 */
  { "D-chunk1ms",    1u, PRIO_LOAD_HIGH,   20u },  /* 1ms 청크 */
};
#define CFG_N          (sizeof(s_cfg) / sizeof(s_cfg[0]))
#define CFG_BATCHES     4u   /* config당 ACT 배치 수 (4 x 6.4초 = 25.6초, 512이벤트) */

static volatile uint8_t s_cfg_idx;   /* 현재 config. 부하/센서 태스크가 읽는다 */

/* ------------------------------------------------------------------ 로그 큐 */

typedef enum
{
  APP_MSG_ACT = 0,   /* 액션 지연 배치 */
  APP_MSG_SEN,       /* 센서 주기 배치 */
  APP_MSG_INF,       /* 합성 부하 잡 소요시간 배치 (결정성의 대가= 처리량) */
  APP_MSG_ERR,       /* 오류 통지 (aux0 = 코드) */
} app_msg_type_t;

typedef struct
{
  uint8_t   type;      /* app_msg_type_t */
  uint8_t   cfg;       /* 이 배치가 속한 스윕 config 인덱스 */
  uint16_t  n;
  uint32_t  batch;
  uint32_t *buf;       /* 원본 사이클 버퍼 - 로그 태스크가 제자리 정렬(파괴) */
  volatile uint8_t *owner;  /* 버퍼 소유권 플래그. 로그가 다 쓰면 0으로 되돌린다 */
  uint32_t  aux0;      /* ACT: 마감 초과 수 / SEN: 읽기 실패 수 / ERR: 코드 */
  uint32_t  aux1;      /* ACT: 이벤트 유실 수(coalesced) */
  uint32_t  aux2;      /* 로그 백프레셔로 측정에서 빠진 샘플 수 */
  uint32_t  aux3;      /* SEN: 지금까지 완성된 윈도우 수 */
  int16_t   ax, ay, az;/* SEN: 생존 신호용 최근 샘플 (가속도) */
  int16_t   gx, gy, gz;/* SEN: 생존 신호용 최근 샘플 (자이로) */
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
static TaskHandle_t s_load_task;    /* config 전환 시 우선순위를 바꾼다 */

/* ------------------------------------------------------------------ 측정 버퍼 */

/* 더블 버퍼: 한쪽을 로그 태스크가 정렬하는 동안 반대쪽에 계속 쌓는다.
 * busy 플래그가 소유권을 표시한다 - 이게 없으면 로그가 밀렸을 때 생산자가
 * 두 바퀴 돌아 "아직 정렬 중인 버퍼"를 덮어쓴다. */
static uint32_t s_act_lat[2][ACT_BATCH_N];
static uint32_t s_sen_int[2][SEN_BATCH_N];
static uint32_t s_inf_job[2][INF_BATCH_N];
static volatile uint8_t s_act_busy[2];
static volatile uint8_t s_sen_busy[2];
static volatile uint8_t s_inf_busy[2];

/* W3 대비: 추론 입력 윈도우 링버퍼 (지금은 쌓기만 한다).
 * 6축 raw int16 = 12B/샘플, 128샘플 = 1536B. g/dps 환산과 특징 추출은 W3에서
 * 윈도우를 뽑는 시점에 한다. */
static mpu6050_imu_t     s_win[WIN_N];
static volatile uint32_t s_win_wr;      /* 총 기록 샘플 수 (인덱스는 & (WIN_N-1)) */
static volatile uint32_t s_win_ready;   /* hop마다 1 증가 = 완성된 윈도우 수 */

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
/**
  * config 전환. 액션 태스크(스윕의 주인)만 부른다.
  * vTaskPrioritySet은 즉시 효력이 있어서, 부하 태스크가 연산 중이어도
  * 다음 스케줄링 지점부터 새 배치가 적용된다.
  */
static void prv_sweep_apply(uint8_t idx)
{
  s_cfg_idx = idx;
  vTaskPrioritySet(s_load_task, s_cfg[idx].prio);
}

static void prv_action_task(void *arg)
{
  uint32_t warm = 0u, idx = 0u, batch = 0u;
  uint32_t miss = 0u, lost = 0u, skip = 0u;
  uint32_t cfg_batch = 0u;
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
        .type = APP_MSG_ACT, .cfg = s_cfg_idx, .n = ACT_BATCH_N, .batch = batch,
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

      /* config를 CFG_BATCHES마다 넘긴다. 전환 직후 WARMUP_N개는 다시 버린다 -
       * 전환 시점의 이벤트는 이전 config의 잡 중간에 걸려 있을 수 있다. */
      if (++cfg_batch >= CFG_BATCHES)
      {
        cfg_batch = 0u;
        prv_sweep_apply((uint8_t)((s_cfg_idx + 1u) % CFG_N));
        warm = 0u;
      }
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
  mpu6050_imu_t s = { 0, 0, 0, 0, 0, 0 };

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

    if (mpu6050_read_imu(&hi2c1, &s))
    {
      s_win[s_win_wr & (WIN_N - 1u)] = s;   /* W3 추론 입력 윈도우 */
      s_win_wr++;

      /* 첫 윈도우는 버퍼가 다 찬 뒤(WIN_N), 이후로는 hop마다 하나씩 완성된다 */
      if ((s_win_wr >= WIN_N) && (((s_win_wr - WIN_N) % WIN_HOP) == 0u))
      {
        s_win_ready++;   /* W3: 여기서 추론 태스크를 깨우게 된다 */
      }
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
        .type = APP_MSG_SEN, .cfg = s_cfg_idx, .n = SEN_BATCH_N, .batch = batch,
        .buf = s_sen_int[side], .owner = &s_sen_busy[side],
        .aux0 = fails, .aux2 = skip, .aux3 = s_win_ready,
        .ax = s.ax, .ay = s.ay, .az = s.az,
        .gx = s.gx, .gy = s.gy, .gz = s.gz,
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

/* ------------------------------------------------------------------ 합성 부하 */

/**
  * 합성 부하 태스크 - **추론이 아니다.**
  *
  * LOAD_PERIOD_MS마다 "잡"을 하나 돌린다. 한 잡의 순수 연산량은 config와
  * 무관하게 LOAD_BUSY_MS로 고정이고, 달라지는 건 그걸 몇 조각으로 쪼개
  * 사이에 블록하느냐뿐이다. 그래서 config끼리 CPU 부하율이 같고
  * (20/53 = 37.7%), 비교에 남는 차이는 선점 구조뿐이다.
  *
  * 잡 소요시간(t_end - t_start)을 같이 잰다. 이게 청킹의 대가다 -
  * 액션 지연을 사는 값으로 이 태스크의 완료시간을 지불한다.
  * 결정성 vs 처리량이 여기서 두 개의 숫자가 된다.
  */
static void prv_load_task(void *arg)
{
  volatile uint32_t acc = 1u;
  const uint32_t busy_cyc = (SystemCoreClock / 1000u) * LOAD_BUSY_MS;

  TickType_t last;
  uint32_t idx = 0u, batch = 0u, warm = 0u, skip = 0u;
  uint8_t  side = 0u, cfg_seen = 0xFFu;

  (void)arg;

  last = xTaskGetTickCount();

  for (;;)
  {
    const sweep_cfg_t *c;
    uint32_t chunk_cyc, t0, job, k;
    uint8_t  cfg_now;

    vTaskDelayUntil(&last, pdMS_TO_TICKS(LOAD_PERIOD_MS));

    cfg_now = s_cfg_idx;
    c = &s_cfg[cfg_now];

    if (cfg_now != cfg_seen)
    {
      cfg_seen = cfg_now;      /* config가 바뀌면 쌓던 배치를 버리고 다시 시작 */
      idx = 0u; warm = 0u; skip = 0u;
    }

    if (c->load_on == 0u)
    {
      continue;                /* 대조군: 잡 자체가 없다 */
    }

    chunk_cyc = busy_cyc / c->chunks;
    t0 = dwt_now();

    for (k = 0u; k < c->chunks; k++)
    {
      uint32_t ck = dwt_now();
      while (dwt_elapsed(ck) < chunk_cyc)
      {
        acc = acc * 1664525u + 1013904223u;   /* 최적화로 안 사라지는 더미 연산 */
      }
      if ((k + 1u) < c->chunks)
      {
        /* 청크 경계. taskYIELD()가 아니라 블로킹이어야 한다 - 이 태스크가
         * 액션보다 높은 우선순위인 config에서는 yield해봐야 스케줄러가
         * 자기 자신을 다시 고른다. */
        vTaskDelay(1u);
      }
    }

    job = dwt_now() - t0;

    if (warm < WARMUP_N)
    {
      warm++;
      continue;
    }
    if (s_inf_busy[side] != 0u)
    {
      skip++;
      continue;
    }

    s_inf_job[side][idx++] = job;
    if (idx >= INF_BATCH_N)
    {
      app_msg_t m = {
        .type = APP_MSG_INF, .cfg = cfg_now, .n = INF_BATCH_N, .batch = batch,
        .buf = s_inf_job[side], .owner = &s_inf_busy[side],
        .aux2 = skip,
      };
      s_inf_busy[side] = 1u;
      if (xQueueSend(s_logq, &m, 0u) != pdTRUE)
      {
        s_inf_busy[side] = 0u;
      }
      side ^= 1u;
      idx = 0u; skip = 0u;
      batch++;
    }
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
  const char *cfg;

  (void)arg;

  printf("\r\n=== W3 sweep up: action(%u) sensor(%u) log(%u) load(%u/%u) ===\r\n",
         (unsigned)PRIO_ACTION, (unsigned)PRIO_SENSOR, (unsigned)PRIO_LOG,
         (unsigned)PRIO_LOAD_LOW, (unsigned)PRIO_LOAD_HIGH);
  printf("event=%lums deadline=%lums(관측 기준선) batch: act=%lu sen=%lu inf=%lu warmup=%lu\r\n",
         (unsigned long)EVENT_PERIOD_MS, (unsigned long)DEADLINE_MS,
         (unsigned long)ACT_BATCH_N, (unsigned long)SEN_BATCH_N,
         (unsigned long)INF_BATCH_N, (unsigned long)WARMUP_N);
  printf("load: busy=%lums period=%lums (=%lu%%, 전 config 동일) cfg마다 %lu배치\r\n",
         (unsigned long)LOAD_BUSY_MS, (unsigned long)LOAD_PERIOD_MS,
         (unsigned long)(LOAD_BUSY_MS * 100u / LOAD_PERIOD_MS),
         (unsigned long)CFG_BATCHES);
  for (uint32_t i = 0u; i < CFG_N; i++)
  {
    printf("  cfg[%lu] %-11s load=%s prio=%-2u chunks=%lu\r\n",
           (unsigned long)i, s_cfg[i].name, s_cfg[i].load_on ? "on " : "off",
           (unsigned)s_cfg[i].prio, (unsigned long)s_cfg[i].chunks);
  }
  printf("tag,config,n,min,median,p99,max\r\n");

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
    cfg = s_cfg[m.cfg].name;

    if (m.type == APP_MSG_ACT)
    {
      uint32_t med10 = prv_cyc_to_us10(st.med);
      uint32_t p9910 = prv_cyc_to_us10(st.p99);
      uint32_t max10 = prv_cyc_to_us10(st.max);
      printf("[ACT] cfg=%-11s batch=%lu n=%u lat_us med=%lu.%lu p99=%lu.%lu max=%lu.%lu"
             " miss(%lums)=%lu lost=%lu skip=%lu\r\n",
             cfg, (unsigned long)m.batch, (unsigned)m.n,
             (unsigned long)(med10 / 10u), (unsigned long)(med10 % 10u),
             (unsigned long)(p9910 / 10u), (unsigned long)(p9910 % 10u),
             (unsigned long)(max10 / 10u), (unsigned long)(max10 % 10u),
             (unsigned long)DEADLINE_MS, (unsigned long)m.aux0,
             (unsigned long)m.aux1, (unsigned long)m.aux2);
      dwt_csv_row_cfg("action_latency", cfg, &st);
    }
    else if (m.type == APP_MSG_INF)
    {
      /* 잡 소요시간은 ms 단위가 읽기 쉽다 (20~40ms 범위) */
      printf("[INF] cfg=%-11s batch=%lu n=%u job_us med=%lu p99=%lu max=%lu skip=%lu\r\n",
             cfg, (unsigned long)m.batch, (unsigned)m.n,
             (unsigned long)(prv_cyc_to_us10(st.med) / 10u),
             (unsigned long)(prv_cyc_to_us10(st.p99) / 10u),
             (unsigned long)(prv_cyc_to_us10(st.max) / 10u),
             (unsigned long)m.aux2);
      dwt_csv_row_cfg("load_job", cfg, &st);
    }
    else
    {
      uint32_t med10 = prv_cyc_to_us10(st.med) / 10u;   /* 주기는 us 정수면 충분 */
      printf("[SEN] cfg=%-11s batch=%lu n=%u interval_us med=%lu p99=%lu max=%lu"
             " fail=%lu skip=%lu win=%lu a=(%d,%d,%d) g=(%d,%d,%d)\r\n",
             cfg, (unsigned long)m.batch, (unsigned)m.n,
             (unsigned long)med10,
             (unsigned long)(prv_cyc_to_us10(st.p99) / 10u),
             (unsigned long)(prv_cyc_to_us10(st.max) / 10u),
             (unsigned long)m.aux0, (unsigned long)m.aux2,
             (unsigned long)m.aux3,
             (int)m.ax, (int)m.ay, (int)m.az,
             (int)m.gx, (int)m.gy, (int)m.gz);
      dwt_csv_row_cfg("imu_interval_task", cfg, &st);
    }

    if (m.owner != NULL)
    {
      *m.owner = 0u;   /* 버퍼 소유권 반환 - 이제 생산자가 다시 써도 된다 */
    }
  }
}

/* ------------------------------------------------------------------ 오버플로 훅 */

/* 훅이 잡은 태스크 이름. 디버거를 붙이면 여기부터 본다. */
volatile const char *g_stack_overflow_task;

/**
  * 스택 오버플로 훅 (configCHECK_FOR_STACK_OVERFLOW=2).
  * 여기서 printf를 부르지 않는다 - 이미 스택이 깨진 상태라 더 깊이 들어가면
  * 원인 정보까지 잃는다. 인터럽트를 끄고 LD2를 고속 점멸시켜 정상 동작(10Hz)과
  * 눈으로 구분되게 한다.
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  g_stack_overflow_task = pcTaskName;

  taskDISABLE_INTERRUPTS();
  for (;;)
  {
    /* 1초에 한 번 토글. 정상 동작은 50ms 토글(초당 20번)이라 눈으로 바로 갈린다.
     * 지연은 DWT로 잰다 - 인터럽트를 껐으니 HAL_Delay(SysTick/TIM10 기반)는 멎는다. */
    uint32_t t0 = dwt_now();
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    while (dwt_elapsed(t0) < SystemCoreClock)
    {
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

  s_cfg_idx = 0u;   /* 스윕은 대조군(L0-noload)부터 시작한다 */

  (void)xTaskCreateStatic(prv_log_task, "log", STK_LOG, NULL,
                          PRIO_LOG, s_stk_log, &s_tcb_log);
  s_load_task = xTaskCreateStatic(prv_load_task, "load", STK_INFER, NULL,
                                  s_cfg[0].prio, s_stk_infer, &s_tcb_infer);
  (void)xTaskCreateStatic(prv_sensor_task, "sensor", STK_SENSOR, NULL,
                          PRIO_SENSOR, s_stk_sensor, &s_tcb_sensor);
  s_action_task = xTaskCreateStatic(prv_action_task, "action", STK_ACTION, NULL,
                                    PRIO_ACTION, s_stk_action, &s_tcb_action);
  configASSERT(s_action_task != NULL);
  configASSERT(s_load_task != NULL);
}
