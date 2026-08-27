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
#include "ae_infer.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ 파라미터 */

#define PRIO_ACTION        48u
#define PRIO_SENSOR        40u
#define PRIO_LOG           16u

/* 진짜 추론 태스크(W3 축 B). 로그(16)보다 아래인 건 system-design.md 1-1의
 * "로그를 추론보다 위에 둔다"는 결정을 유지하는 것이고, 합성 부하의 LOW(8)보다
 * 위인 건 **실제 일이 더미 부하보다 아래로 밀리면 안 되기 때문**이다.
 * 추론은 실측상 CPU의 0.2%라 이 배치가 축 A 스윕을 흔들지 않는다 - 오히려
 * 같은 8로 두면 부하 태스크와 라운드로빈이 돌아 부하 잡 완료시간에 잡음이 낀다. */
#define PRIO_INFER         12u

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
 * 어긋나면 학습된 특징 분포와 온보드 특징이 다른 것을 재게 된다. (2^n 필수)
 * 값 자체는 model_weights.h가 굽는 AE_WIN_LEN/AE_HOP이 원본이고, 아래 정적 단언이
 * 둘이 갈라지는 걸 막는다. */
#define WIN_N             128u
#define WIN_HOP            64u

_Static_assert(WIN_N == AE_WIN_LEN, "윈도우 길이가 model_weights.h와 다르다");
_Static_assert(WIN_HOP == AE_HOP,   "hop이 model_weights.h와 다르다");
_Static_assert(1000u / SENSOR_PERIOD_MS == AE_FS_HZ,
               "센서 주기가 모델 학습 샘플레이트와 다르다");

/* 추론 배치. 윈도우가 hop마다 = 0.64초마다 하나씩 나오므로 128개면 81.9초다.
 * 이 프로젝트 규칙(반복 >= 100)을 지키면서 스윕 한 바퀴(128초)와 얼추 맞는 크기.
 * 그동안 화면이 조용하면 죽은 것처럼 보이므로, 윈도우마다 한 줄씩 따로 찍는다
 * (그 printf는 로그 태스크에서 도니 측정 구간에 안 섞인다). */
#define AE_BATCH_N        128u

/* 이상 판정 디바운스. 윈도우 한 개로 액션을 걸면 스파이크 하나에 오경보가 난다.
 * 2개 연속(= 0.64초 간격 두 번, 50% 겹침이라 실제로는 1.92초 구간)을 요구한다.
 * 대가는 감지 지연 +0.64초 - 윈도우 자체가 이미 1.28초라 비율로는 작다. */
#define AE_DEBOUNCE_N       2u

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
  APP_MSG_EDGE,      /* 윈도우 1개의 추론 결과 (축 B 데모의 눈에 보이는 부분) */
  APP_MSG_AECYC,     /* 추론 파이프라인 사이클 배치 -> inference_cycles.csv */
  APP_MSG_AELAT,     /* 윈도우 완성 -> 판정 지연 배치 */
  APP_MSG_STACK,     /* 스택 high-water 보고 요청 (aux0 = 완료된 스윕 바퀴 수) */
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
  uint32_t  aux3;      /* SEN: 지금까지 완성된 윈도우 수 / EDGE: 판정 지연(사이클) */
  uint32_t  aux4;      /* SEN: 포화 근접 샘플 수 / EDGE: 추론 소요(사이클) */
  int32_t   err;       /* EDGE: 재구성 오차 (int8 경로) */
  uint8_t   anom;      /* EDGE: 이상 판정 (디바운스 후) */
  uint8_t   edge_flag; /* EDGE: 이번 윈도우에서 판정이 전환됐나 = 로컬 액션 발동 */
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
#define STK_LOAD   192u   /* 합성 부하 - busy 루프뿐이라 얕다 */

/* 추론 태스크. 큰 버퍼(윈도우 float 3KB, 층간 핑퐁)는 전부 파일 스코프 static이라
 * 여기 안 쌓인다. 320워드(1,280B)는 여유를 둔 값이고, W4에서 high-water로 확정한다. */
#define STK_INFER  320u

static StaticTask_t s_tcb_action, s_tcb_sensor, s_tcb_log, s_tcb_load, s_tcb_infer;
static StackType_t  s_stk_action[STK_ACTION];
static StackType_t  s_stk_sensor[STK_SENSOR];
static StackType_t  s_stk_log[STK_LOG];
static StackType_t  s_stk_load[STK_LOAD];
static StackType_t  s_stk_infer[STK_INFER];

static TaskHandle_t s_action_task;
static TaskHandle_t s_load_task;    /* config 전환 시 우선순위를 바꾼다 */
static TaskHandle_t s_infer_task;   /* 센서 태스크가 윈도우 완성 시 깨운다 */
static TaskHandle_t s_sensor_task;  /* 스택 high-water 조회용 (W4) */
static TaskHandle_t s_log_task;     /* 스택 high-water 조회용 (W4) */

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

/* 추론 입력 윈도우 링버퍼. 6축 raw int16 = 12B/샘플, 128샘플 = 1536B.
 * g/dps 환산은 추론 태스크가 스냅숏에서 한다 - 링에 float로 담으면 4KB가 된다. */
static mpu6050_imu_t     s_win[WIN_N];
static volatile uint32_t s_win_wr;      /* 총 기록 샘플 수 (인덱스는 & (WIN_N-1)) */
static volatile uint32_t s_win_ready;   /* hop마다 1 증가 = 완성된 윈도우 수 */

/* 윈도우 스냅숏 - **센서 태스크가** 채운다.
 *
 * 추론 태스크가 링을 직접 읽으면 안 된다. 링 크기가 WIN_N과 같아서, 윈도우가
 * 완성된 순간부터 10ms 뒤에는 가장 오래된 샘플이 이미 덮인다. 추론 태스크는
 * 우선순위 12라 config B(부하 50)에서는 20ms 넘게 못 깨어날 수 있고, 그러면
 * 윈도우 앞부분과 뒷부분이 서로 다른 시각의 데이터로 찢어진다.
 * 그래서 소유권을 생산자 쪽에 두고, 완성 즉시 센서 태스크가 복사한다
 * (1,536B memcpy, 0.64초에 한 번 - 10ms 예산에 영향 없다).
 *
 * busy=1이면 추론이 아직 이전 것을 들고 있다는 뜻이라 이번 윈도우는 버린다
 * (s_win_drop). 조용히 삼키지 않고 세는 건 1-9의 실패 모드 원칙 그대로다. */
static mpu6050_imu_t     s_snap[WIN_N];
static volatile uint8_t  s_snap_busy;
static volatile uint32_t s_snap_t0;     /* 윈도우 완성 시각 (판정 지연의 기준점) */
static volatile uint32_t s_win_drop;    /* 추론이 못 따라가 버린 윈도우 수 */

/* 스냅숏 -> 물리 단위 float. [128][6] 채널 인터리브 = ae_features의 입력 계약.
 * 3,072B라 태스크 스택에 못 올린다. 추론 태스크만 만진다(재진입 없음). */
static float s_winf[WIN_N * AE_N_CH];

/* 추론 측정 버퍼. 더블버퍼는 안 쓴다 - 윈도우가 0.64초에 하나라, 로그가
 * 배치 하나를 정렬하는 동안(수 ms) 다음 배치가 찰 일이 원리적으로 없다.
 * 대신 소유권 플래그는 그대로 둬서 만약을 세게 한다. */
static uint32_t s_ae_cyc[AE_BATCH_N];   /* 파이프라인 전체 소요 (특징+양자화+순전파) */
static uint32_t s_ae_lat[AE_BATCH_N];   /* 윈도우 완성 -> 판정 완료 */
static volatile uint8_t s_ae_busy;

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
  uint32_t cfg_batch = 0u, sweeps = 0u;
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

        /* 스윕이 한 바퀴(전 config, 128초) 돌 때마다 스택 high-water 보고를
         * 요청한다(W4 memory_footprint). 한 바퀴를 기다리는 이유: high-water는
         * "지금까지의 최악"이라 모든 config의 최악 경로를 지난 뒤라야 의미가
         * 있다. 바퀴마다 반복해 찍는 건 값이 수렴하는지 보기 위해서다 -
         * 늘고 있으면 아직 최악을 못 본 것이다. 조회/printf는 로그 태스크 몫. */
        if (s_cfg_idx == 0u)
        {
          app_msg_t sm = { .type = APP_MSG_STACK, .aux0 = ++sweeps };
          (void)xQueueSend(s_logq, &sm, 0u);   /* 큐 포화면 다음 바퀴에 또 온다 */
        }
      }
    }
  }
}

/* ------------------------------------------------------------------ 센서 태스크 */

/**
  * MPU6050을 vTaskDelayUntil로 100Hz 폴링. 실제 주기를 DWT로 재서
  * "태스크 환경에서도 100Hz가 유지되는가"를 배치 통계로 보고한다.
  */
/**
  * 완성된 윈도우를 링에서 펴서 스냅숏으로 복사한다. **센서 태스크에서만** 부른다.
  *
  * 링이 가득 찬 상태이므로 "다음에 쓸 자리"가 곧 가장 오래된 샘플이다.
  * 거기서 끝까지 한 번, 처음부터 거기까지 한 번 - 두 memcpy로 시간순이 된다.
  */
static void prv_snapshot_window(void)
{
  const uint32_t oldest = s_win_wr & (WIN_N - 1u);
  const uint32_t tail = WIN_N - oldest;

  memcpy(&s_snap[0], &s_win[oldest], tail * sizeof(mpu6050_imu_t));
  if (oldest != 0u)
  {
    memcpy(&s_snap[tail], &s_win[0], oldest * sizeof(mpu6050_imu_t));
  }
  s_snap_t0 = dwt_now();
}

static void prv_sensor_task(void *arg)
{
  TickType_t last;
  uint32_t prev, warm = 0u, idx = 0u, batch = 0u, fails = 0u, skip = 0u;
  uint32_t sat = 0u, run_fail = 0u;
  uint8_t  side = 0u;
  mpu6050_imu_t s = { 0, 0, 0, 0, 0, 0 };

  (void)arg;

  /* 브링업이 이미 깨웠어도 무해(멱등). 브링업을 건너뛴 부팅도 여기서 복구 */
  (void)mpu6050_wake(&hi2c1);
  (void)mpu6050_configure(&hi2c1, NULL);   /* 레인지/DLPF도 같은 이유로 재적용 */

  last = xTaskGetTickCount();
  prev = dwt_now();

  for (;;)
  {
    uint32_t t0;

    vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSOR_PERIOD_MS));

    t0 = dwt_now();

    if (mpu6050_read_imu(&hi2c1, &s))
    {
      run_fail = 0u;
      /* 포화 근접 샘플을 센다. "+-2g로 충분한가"를 추측이 아니라 데이터로
       * 답하기 위한 카운터다 - 0이 아니면 클리핑이 특징을 망치고 있다는 뜻이고,
       * 그때 AFS_SEL을 올린다(mpu6050.h 주석). */
      if (mpu6050_accel_saturated(&s) || mpu6050_gyro_saturated(&s))
      {
        sat++;
      }

      s_win[s_win_wr & (WIN_N - 1u)] = s;   /* 추론 입력 윈도우 */
      s_win_wr++;

      /* 첫 윈도우는 버퍼가 다 찬 뒤(WIN_N), 이후로는 hop마다 하나씩 완성된다 */
      if ((s_win_wr >= WIN_N) && (((s_win_wr - WIN_N) % WIN_HOP) == 0u))
      {
        s_win_ready++;

        if (s_snap_busy == 0u)
        {
          prv_snapshot_window();
          s_snap_busy = 1u;              /* 소유권을 추론 태스크로 넘긴다 */
          (void)xTaskNotifyGive(s_infer_task);
        }
        else
        {
          s_win_drop++;                  /* 추론이 아직 이전 윈도우를 들고 있다 */
        }
      }
    }
    else
    {
      fails++;
      /* 수집 중 실측(2026-08-08)에서 배선이 흔들리자 버스가 잠긴 채 5,000샘플이
       * 통째로 날아갔다. 여기서도 같은 일이 나면 센서가 영영 안 돌아온다 -
       * 스케줄러가 도는 중이라 리셋 말고는 손쓸 방법이 없어진다. */
      if (++run_fail >= 5u)
      {
        run_fail = 0u;
        (void)mpu6050_recover(&hi2c1);   /* 수 ms 블로킹. 우선순위 40이라 액션은 안 문다 */
      }
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
        .aux0 = fails, .aux2 = skip, .aux3 = s_win_ready, .aux4 = sat,
        .ax = s.ax, .ay = s.ay, .az = s.az,
        .gx = s.gx, .gy = s.gy, .gz = s.gz,
      };
      s_sen_busy[side] = 1u;
      if (xQueueSend(s_logq, &m, 0u) != pdTRUE)
      {
        s_sen_busy[side] = 0u;
      }
      side ^= 1u;
      idx = 0u; fails = 0u; skip = 0u; sat = 0u;
      batch++;
    }
  }
}

/* ------------------------------------------------------------------ 추론 태스크 */

/**
  * 스냅숏(raw int16) -> 물리 단위 float, 채널 인터리브.
  *
  * 여기서 g/dps로 바꾸는 이유: 모델의 정규화 상수(ae_norm_mean/std)가 물리 단위
  * 기준이라 LSB 그대로 넣으면 z-score가 통째로 어긋난다. 그리고 이렇게 두면
  * **풀스케일 레인지를 바꿔도 모델을 다시 학습할 필요가 없다** - 환산 상수만
  * 바뀌고 그 위 층은 그대로다 (model/config.py의 그 약속이 여기서 지켜진다).
  */
static void prv_window_to_float(void)
{
  const float ka = 1.0f / (float)MPU6050_ACCEL_LSB_PER_G;
  const float kg = 1.0f / MPU6050_GYRO_LSB_PER_DPS;
  uint32_t i;

  for (i = 0u; i < WIN_N; i++)
  {
    float *o = &s_winf[i * AE_N_CH];
    o[0] = (float)s_snap[i].ax * ka;
    o[1] = (float)s_snap[i].ay * ka;
    o[2] = (float)s_snap[i].az * ka;
    o[3] = (float)s_snap[i].gx * kg;
    o[4] = (float)s_snap[i].gy * kg;
    o[5] = (float)s_snap[i].gz * kg;
  }
}

/**
  * 축 B의 본체: 센서 -> 로컬 추론 -> 로컬 액션.
  *
  * 측정 두 가지를 같이 낸다.
  *   (1) 파이프라인 사이클 - 환산+특징+양자화+순전파. 실제 태스크 환경이라
  *       상위 태스크 선점이 꼬리에 섞인다. 선점 없는 순수 커널 비용은 부팅 시
  *       ae_bench()가 따로 잰다. **둘은 다른 질문에 답하는 다른 숫자다.**
  *   (2) 윈도우 완성 -> 판정 완료 지연. 축 B판 "이벤트 -> 액션"이고,
  *       config에 따라 이게 얼마나 벌어지는지가 축 A와 축 B를 잇는 지점이다.
  *
  * 로컬 액션은 판정이 **전환될 때** 발동한다(디바운스 AE_DEBOUNCE_N). LD2는
  * 마감시한 태스크가 20Hz로 쓰고 있어서 여기서 못 쓴다 - 이 데모의 액추에이터는
  * UART 경보다. 액추에이터가 무엇이든 판정까지의 경로와 지연은 동일하다.
  */
static void prv_infer_task(void *arg)
{
  uint32_t idx = 0u, batch = 0u, skip = 0u, bdrop = 0u, nwin = 0u;
  uint32_t run = 0u;          /* 현재 판정이 연속으로 나온 횟수 (디바운스) */
  uint8_t  raw = 0u;          /* 이번 윈도우의 생 판정 */
  uint8_t  state = 0u;        /* 디바운스를 통과한 확정 상태 */
  uint32_t acts = 0u;         /* 로컬 액션 발동 횟수 */

  (void)arg;

  for (;;)
  {
    int32_t  err;
    uint32_t t0, t_snap, cyc, lat;
    uint8_t  flipped = 0u;

    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* 기준 시각을 **소유권을 놓기 전에** 복사해 둔다. 반납한 뒤에 s_snap_t0를 읽으면,
     * 그 사이 센서 태스크(우선순위 40)가 다음 스냅숏을 떠서 값을 덮을 수 있다.
     * 그러면 지연이 엉뚱하게 작게(또는 unsigned 언더플로로 거대하게) 나온다.
     * 창은 수 마이크로초라 드물게 터지지만, 드물게 터지는 게 정확히 max를 망친다. */
    t_snap = s_snap_t0;

    /* --- 측정 구간 시작. 여기서 printf/큐 전송을 하지 않는다 --- */
    t0 = dwt_now();
    prv_window_to_float();
    err = ae_run_window(s_winf, (int)WIN_N);
    cyc = dwt_elapsed(t0);
    /* --- 측정 구간 끝 --- */

    s_snap_busy = 0u;          /* 스냅숏 소유권 반납 - 다음 윈도우를 받을 수 있다 */

    raw = (uint8_t)(ae_is_anomaly(err) ? 1u : 0u);
    if (raw == state)
    {
      run = 0u;                /* 현재 상태와 같으면 전환 후보가 리셋된다 */
    }
    else if (++run >= AE_DEBOUNCE_N)
    {
      state = raw;             /* 연속 N회 반대로 나왔다 -> 상태 전환 = 로컬 액션 */
      run = 0u;
      flipped = 1u;
      acts++;
    }

    /* 지연은 액션(=판정 확정)까지다. 큐 전송은 그 뒤라 여기 안 들어간다. */
    lat = dwt_now() - t_snap;
    nwin++;

    /* 윈도우마다 한 줄 - 데모의 눈에 보이는 부분이자 생존 신호.
     * 0.64초에 한 번이고 printf는 로그 태스크에서 도니 위 측정과 무관하다. */
    {
      app_msg_t m = {
        .type = APP_MSG_EDGE, .cfg = s_cfg_idx, .batch = nwin,
        .aux0 = s_win_drop, .aux3 = lat, .aux4 = cyc,
        .err = err, .anom = state, .edge_flag = flipped,
      };
      (void)xQueueSend(s_logq, &m, 0u);   /* 드롭돼도 무해 - 통계는 아래 배치가 낸다 */
    }

    if (s_ae_busy != 0u)
    {
      skip++;
      continue;
    }

    s_ae_cyc[idx] = cyc;
    s_ae_lat[idx] = lat;
    idx++;

    if (idx >= AE_BATCH_N)
    {
      app_msg_t mc = {
        .type = APP_MSG_AECYC, .cfg = s_cfg_idx, .n = AE_BATCH_N, .batch = batch,
        .buf = s_ae_cyc, .owner = NULL,
        .aux0 = acts, .aux2 = skip, .aux3 = bdrop,
      };
      app_msg_t ml = {
        .type = APP_MSG_AELAT, .cfg = s_cfg_idx, .n = AE_BATCH_N, .batch = batch,
        .buf = s_ae_lat, .owner = &s_ae_busy, .aux2 = skip,
      };

      /* 두 배열이라 메시지도 둘이다. 소유권 반납은 **뒤엣것**에만 건다 -
       * 로그 태스크가 FIFO로 처리하므로 뒤엣것이 끝났다면 앞엣것도 끝났다.
       *
       * 자리를 먼저 확인하는 건 흔한 경우에 둘을 같이 넣기 위해서지, 원자성을
       * 보장하지는 않는다 - 확인과 전송 사이에 상위 태스크가 큐를 채울 수 있다.
       * 그래서 **실패 경로를 반드시 닫는다.** ml이 못 들어가면 소유권을 놓아줄
       * 주체가 없어져 s_ae_busy가 영영 1로 남고, 그 뒤 모든 배치가 조용히 막힌다.
       *
       * mc만 들어간 채 busy를 푸는 게 이론상 "로그가 정렬 중인 배열에 덮어쓰기"인데,
       * 정렬은 ms 단위고 다음 배치가 차는 데는 82초 걸린다. 도달 불가능한 창이다. */
      if (uxQueueSpacesAvailable(s_logq) >= 2u)
      {
        s_ae_busy = 1u;
        if ((xQueueSend(s_logq, &mc, 0u) != pdTRUE) ||
            (xQueueSend(s_logq, &ml, 0u) != pdTRUE))
        {
          s_ae_busy = 0u;
          bdrop++;
        }
      }
      else
      {
        bdrop++;               /* 큐 포화 - 이 배치는 통째로 못 나갔다 */
      }
      idx = 0u; skip = 0u;
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

  printf("\r\n=== W3 up: action(%u) sensor(%u) log(%u) infer(%u) load(%u/%u) ===\r\n",
         (unsigned)PRIO_ACTION, (unsigned)PRIO_SENSOR, (unsigned)PRIO_LOG,
         (unsigned)PRIO_INFER, (unsigned)PRIO_LOAD_LOW, (unsigned)PRIO_LOAD_HIGH);
  printf("edge: win=%lu hop=%lu (%lums/추론) debounce=%lu batch=%lu (%lu초)\r\n",
         (unsigned long)WIN_N, (unsigned long)WIN_HOP,
         (unsigned long)(WIN_HOP * SENSOR_PERIOD_MS), (unsigned long)AE_DEBOUNCE_N,
         (unsigned long)AE_BATCH_N,
         (unsigned long)(AE_BATCH_N * WIN_HOP * SENSOR_PERIOD_MS / 1000u));
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

    cfg = s_cfg[m.cfg].name;

    /* 스택 high-water: 태스크별 "여태 가장 얕게 남았던 잔여 스택"(워드, 1워드=4B).
     * 여기(로그 태스크)서 조회하는 이유는 printf 규칙과 같다 - 다른 태스크의
     * 측정 경로에 조회 비용(스택 영역 스캔)을 섞지 않는다. 값은 단조감소라
     * 스윕 바퀴가 거듭돼도 같은 숫자면 수렴한 것이다. */
    if (m.type == APP_MSG_STACK)
    {
      printf("[STK] sweep=%lu 잔여최소/총량(워드):"
             " action=%lu/%u sensor=%lu/%u log=%lu/%u infer=%lu/%u load=%lu/%u\r\n",
             (unsigned long)m.aux0,
             (unsigned long)uxTaskGetStackHighWaterMark(s_action_task), (unsigned)STK_ACTION,
             (unsigned long)uxTaskGetStackHighWaterMark(s_sensor_task), (unsigned)STK_SENSOR,
             (unsigned long)uxTaskGetStackHighWaterMark(s_log_task),    (unsigned)STK_LOG,
             (unsigned long)uxTaskGetStackHighWaterMark(s_infer_task),  (unsigned)STK_INFER,
             (unsigned long)uxTaskGetStackHighWaterMark(s_load_task),   (unsigned)STK_LOAD);
      continue;   /* 배치 메시지가 아니다 (buf == NULL) */
    }

    /* EDGE는 배치가 아니라 윈도우 한 개짜리 결과라 통계를 뽑지 않는다.
     * 오차는 정수라 그대로 찍고, 임계값과 나란히 둬서 판정 근거가 보이게 한다. */
    if (m.type == APP_MSG_EDGE)
    {
      printf("[EDGE]%s win=%-4lu cfg=%-11s err=%-8ld thr=%d %s"
             " infer=%lu.%luus lat=%lu.%luus drop=%lu\r\n",
             m.edge_flag ? "*" : " ",
             (unsigned long)m.batch, cfg, (long)m.err, AE_THRESHOLD_INT,
             m.anom ? "ANOMALY" : "normal ",
             (unsigned long)(prv_cyc_to_us10(m.aux4) / 10u),
             (unsigned long)(prv_cyc_to_us10(m.aux4) % 10u),
             (unsigned long)(prv_cyc_to_us10(m.aux3) / 10u),
             (unsigned long)(prv_cyc_to_us10(m.aux3) % 10u),
             (unsigned long)m.aux0);
      continue;   /* 소유권 반납 대상이 아니다 (buf == NULL) */
    }

    dwt_stats(m.buf, m.n, &st);

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
    else if (m.type == APP_MSG_AECYC)
    {
      uint32_t med10 = prv_cyc_to_us10(st.med);
      uint32_t max10 = prv_cyc_to_us10(st.max);
      printf("[AE ] cfg=%-11s batch=%lu n=%u infer_us med=%lu.%lu max=%lu.%lu"
             " (min=%lu cyc) act=%lu skip=%lu bdrop=%lu\r\n",
             cfg, (unsigned long)m.batch, (unsigned)m.n,
             (unsigned long)(med10 / 10u), (unsigned long)(med10 % 10u),
             (unsigned long)(max10 / 10u), (unsigned long)(max10 % 10u),
             (unsigned long)st.min, (unsigned long)m.aux0,
             (unsigned long)m.aux2, (unsigned long)m.aux3);
      dwt_csv_row_cfg("inference_cycles", cfg, &st);
    }
    else if (m.type == APP_MSG_AELAT)
    {
      uint32_t med10 = prv_cyc_to_us10(st.med);
      uint32_t max10 = prv_cyc_to_us10(st.max);
      printf("[AE ] cfg=%-11s batch=%lu n=%u decide_us med=%lu.%lu max=%lu.%lu\r\n",
             cfg, (unsigned long)m.batch, (unsigned)m.n,
             (unsigned long)(med10 / 10u), (unsigned long)(med10 % 10u),
             (unsigned long)(max10 / 10u), (unsigned long)(max10 % 10u));
      dwt_csv_row_cfg("infer_decide_latency", cfg, &st);
    }
    else
    {
      uint32_t med10 = prv_cyc_to_us10(st.med) / 10u;   /* 주기는 us 정수면 충분 */
      printf("[SEN] cfg=%-11s batch=%lu n=%u interval_us med=%lu p99=%lu max=%lu"
             " fail=%lu skip=%lu sat=%lu win=%lu a=(%d,%d,%d) g=(%d,%d,%d)\r\n",
             cfg, (unsigned long)m.batch, (unsigned)m.n,
             (unsigned long)med10,
             (unsigned long)(prv_cyc_to_us10(st.p99) / 10u),
             (unsigned long)(prv_cyc_to_us10(st.max) / 10u),
             (unsigned long)m.aux0, (unsigned long)m.aux2,
             (unsigned long)m.aux4, (unsigned long)m.aux3,
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

  s_log_task = xTaskCreateStatic(prv_log_task, "log", STK_LOG, NULL,
                                 PRIO_LOG, s_stk_log, &s_tcb_log);
  s_load_task = xTaskCreateStatic(prv_load_task, "load", STK_LOAD, NULL,
                                  s_cfg[0].prio, s_stk_load, &s_tcb_load);
  /* 추론 태스크는 센서보다 **먼저** 만든다 - 센서가 첫 윈도우에서
   * xTaskNotifyGive(s_infer_task)를 부르는데, 그때 핸들이 NULL이면 죽는다. */
  s_infer_task = xTaskCreateStatic(prv_infer_task, "infer", STK_INFER, NULL,
                                   PRIO_INFER, s_stk_infer, &s_tcb_infer);
  s_sensor_task = xTaskCreateStatic(prv_sensor_task, "sensor", STK_SENSOR, NULL,
                                    PRIO_SENSOR, s_stk_sensor, &s_tcb_sensor);
  s_action_task = xTaskCreateStatic(prv_action_task, "action", STK_ACTION, NULL,
                                    PRIO_ACTION, s_stk_action, &s_tcb_action);
  configASSERT(s_action_task != NULL);
  configASSERT(s_load_task != NULL);
  configASSERT(s_infer_task != NULL);
}
