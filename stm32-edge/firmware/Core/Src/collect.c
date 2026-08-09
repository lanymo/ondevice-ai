/**
  ******************************************************************************
  * @file    collect.c
  * @brief   보드 실측 데이터 수집 모드 (스케줄러 전, 단독 실행)
  ******************************************************************************
  */
#include "collect.h"

#include "dwt.h"
#include "mpu6050.h"

#include <stdio.h>

/* 샘플레이트는 model/config.py의 FS_HZ와 **같아야 한다.** 다르면 학습 데이터와
 * 추론 입력의 시간축이 어긋나 std/mad 같은 특징이 통째로 달라진다. */
#define COLLECT_FS_HZ      100u
#define COLLECT_REST_S       8u   /* 세션 사이 준비 시간 */

/* 세션 구성.
 *
 * **세션을 여러 개로 쪼개는 이유**: dataset.py가 윈도우를 세션 경계 너머로 만들지
 * 않는다. 한 클래스를 파일 하나로만 뽑으면 그 세션의 장착 상태·기울기가 곧 클래스
 * 라벨이 되어버린다(model/README.md §1에서 합성 데이터로 이미 한 번 겪은 누수다).
 * 세션마다 손 위치가 조금씩 달라지는 게 오히려 필요한 다양성이다.
 *
 * normal이 anomaly보다 긴 이유 두 가지:
 *   1) 오토인코더는 **정상만으로** 학습한다(비지도). 정상 표본이 많아야 한다.
 *   2) normal은 사람이 아무것도 안 해도 되지만 anomaly는 45초씩 흔들면 팔이 죽는다.
 *
 * 정상 3 x 45s = 135s -> hop 0.64s 기준 약 208 윈도우 (학습 156 / 검증 52).
 * 이상 3 x 25s =  75s -> 약 114 윈도우 (평가용).
 */
typedef struct
{
  const char *label;
  uint16_t    secs;
  const char *hint;    /* 사용자에게 시킬 동작. 라벨과 동작이 갈리면 데이터가 거짓이 된다 */
} collect_session_t;

/* 이상 라벨을 imbalance/impact/looseness로 나눈 이유: 합성 데이터셋과 같은 클래스
 * 이름을 쓰면 evaluate.py의 recall_by_kind 열이 그대로 살아서, 어떤 종류를 못 잡는지
 * 볼 수 있다. dataset.is_anomaly()는 'normal'이 아닌 것을 전부 이상으로 묶으므로
 * 학습·판정에는 영향이 없다 - 진단용 라벨이다. (합성엔 bearing도 있었지만
 * 손으로 베어링 결함의 고주파를 흉내내는 건 정직하지 않아서 뺐다.) */
static const collect_session_t k_sess[] = {
  { "normal",     50u, "손대지 마세요"                     },
  { "imbalance",  25u, "일정한 리듬으로 계속 흔드세요"     },
  { "normal",     50u, "손대지 마세요"                     },
  { "impact",     25u, "1초에 한두 번씩 톡톡 치세요"       },
  { "normal",     50u, "손대지 마세요"                     },
  { "looseness",  25u, "불규칙하게 덜그럭거리듯 흔드세요"  },
  { "normal",     50u, "손대지 마세요"                     },
};
#define SESS_N  (sizeof(k_sess) / sizeof(k_sess[0]))

/* 사용자가 손을 대야 하는 세션인가. LD2 신호에 쓴다.
 * 라벨 문자열 비교 대신 'normal이 아니면 흔든다'로 두면 라벨을 늘려도 안 깨진다. */
static bool prv_is_shake(uint32_t i)
{
  return (k_sess[i].label[0] != 'n');
}

bool collect_requested(void)
{
  /* B1은 GPIO_NOPULL로 잡혀 있고 Nucleo 보드에 외부 풀업이 있다 -> 눌리면 0.
   * EXTI 모드로 설정돼 있어도 입력 레지스터는 그대로 읽힌다.
   * 3회 연속 눌림을 요구하는 건 채터링 때문이 아니라(사람이 누르고 있는 상태다)
   * 부팅 직후 핀이 안정되기 전의 한 번을 걸러내려는 것. */
  for (uint32_t i = 0u; i < 3u; i++)
  {
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_RESET)
    {
      return false;
    }
    HAL_Delay(5);
  }
  return true;
}

/** 남은 초를 카운트다운하며 LD2를 빠르게 점멸(= 준비하라는 신호). */
static void prv_rest(uint32_t secs, const char *next_label, const char *hint)
{
  printf("[COLLECT] %lu초 뒤 시작 -> %s : **%s**\r\n",
         (unsigned long)secs, next_label, hint);

  for (uint32_t s = secs; s > 0u; s--)
  {
    printf("[COLLECT]   %lu...\r\n", (unsigned long)s);
    for (uint32_t k = 0u; k < 5u; k++)   /* 1초를 5번 토글 = 눈에 띄는 점멸 */
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(100);
    }
    HAL_Delay(500);
  }
}

/**
  * 세션 하나. 정확히 100Hz로 n샘플을 뽑아 CSV 한 줄씩 뱉는다.
  *
  * 주기를 HAL_Delay가 아니라 **DWT 절대 시각**으로 잡는 이유: HAL_Delay(10)은
  * 실측 11.00ms(=90.9Hz)였다(measurements/dwt_baseline.csv의 imu_interval_haldelay).
  * +1틱 보장 동작 때문인데, 그대로 쓰면 학습 데이터가 90.9Hz인데 추론은 100Hz인
  * 상태가 된다 - 시간축이 10% 어긋난 데이터로 학습하는 셈이다.
  * 절대 시각 누산이라 printf가 몇 ms 먹어도 다음 샘플 시각이 밀리지 않는다(무드리프트).
  */
static void prv_session(I2C_HandleTypeDef *hi2c, uint32_t idx)
{
  const uint32_t n = (uint32_t)k_sess[idx].secs * COLLECT_FS_HZ;
  const uint32_t period = SystemCoreClock / COLLECT_FS_HZ;
  uint32_t fails = 0u, sat = 0u, late = 0u, written = 0u;
  uint32_t run_fail = 0u, recov = 0u;   /* 연속 실패 / 버스 복구 시도 횟수 */
  uint32_t next = dwt_now();
  mpu6050_imu_t s = { 0, 0, 0, 0, 0, 0 };

  /* 흔드는 세션이면 LD2 계속 켬, 정지 세션이면 끔. 화면 안 봐도 손만 보고 알 수 있게. */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,
                    prv_is_shake(idx) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  /* '#'로 시작하는 줄과 알파벳으로 시작하는 줄은 features.py load_run이 건너뛴다.
   * 그래서 이 마커들이 데이터에 섞여도 파싱이 깨지지 않는다. */
  printf("#SESSION_BEGIN,%lu,%s,%lu,%lu\r\n",
         (unsigned long)idx, k_sess[idx].label,
         (unsigned long)COLLECT_FS_HZ, (unsigned long)n);
  printf("t_ms,ax,ay,az,gx,gy,gz\r\n");

  for (uint32_t i = 0u; i < n; i++)
  {
    next += period;
    /* 부호 있는 비교라 랩어라운드에 안전하다. 이미 지났으면 즉시 통과(late로 센다). */
    if ((int32_t)(dwt_now() - next) >= 0)
    {
      late++;
    }
    while ((int32_t)(dwt_now() - next) < 0)
    {
    }

    if (!mpu6050_read_imu(hi2c, &s))
    {
      fails++;
      /* 연속 실패가 쌓이면 버스가 잠긴 것으로 보고 강제로 푼다. 5회로 잡은 이유:
       * 산발적 1~2회 실패까지 매번 복구(수 ms)를 돌리면 그게 주기를 깬다.
       * 복구는 t_ms 축을 건너뛰게 만들지만, 실패한 샘플은 애초에 안 쓰므로
       * import 쪽에서 t_ms 불연속으로 잘라내면 그만이다. */
      if (++run_fail >= 5u)
      {
        run_fail = 0u;
        recov++;
        (void)mpu6050_recover(hi2c);
        next = dwt_now();   /* 복구에 수 ms 썼으니 주기 기준점을 다시 잡는다 */
      }
      continue;            /* 실패한 샘플은 쓰지 않는다 - 앞 값을 복제하면 std/mad가 거짓이 된다 */
    }
    run_fail = 0u;
    if (mpu6050_accel_saturated(&s) || mpu6050_gyro_saturated(&s))
    {
      sat++;               /* 버리지는 않는다. 세고 보고만 한다 - 판단은 사람이 */
    }

    /* raw LSB로 뱉는다. g/dps 환산은 PC(model/import_board_csv.py)에서.
     * newlib-nano는 %f를 링크에서 빼므로 애초에 float 출력이 안 되고,
     * 정수가 짧아서 UART 대역폭도 아낀다(한 줄 ~4.3ms < 10ms 예산). */
    printf("%lu,%d,%d,%d,%d,%d,%d\r\n",
           (unsigned long)(i * 1000u / COLLECT_FS_HZ),
           (int)s.ax, (int)s.ay, (int)s.az,
           (int)s.gx, (int)s.gy, (int)s.gz);
    written++;
  }

  printf("#SESSION_END,%lu,%s,%lu,%lu,%lu,%lu,%lu\r\n",
         (unsigned long)idx, k_sess[idx].label,
         (unsigned long)written, (unsigned long)fails,
         (unsigned long)sat, (unsigned long)late, (unsigned long)recov);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void collect_run(I2C_HandleTypeDef *hi2c)
{
  uint32_t total = 0u;

  for (uint32_t i = 0u; i < SESS_N; i++)
  {
    total += k_sess[i].secs;
  }

  printf("\r\n=== 수집 모드 (B1이 눌린 채로 부팅됨) ===\r\n");
  printf("센서 설정은 평소 실행과 동일하다 (AFS_SEL=%u, FS_SEL=%u, DLPF_CFG=%u) -\r\n",
         (unsigned)MPU6050_AFS_SEL, (unsigned)MPU6050_FS_SEL,
         (unsigned)MPU6050_DLPF_CFG);
  printf("같은 바이너리에서 분기하므로 학습 데이터와 추론이 같은 설정을 공유한다.\r\n");
  printf("샘플레이트 %luHz, 세션 %lu개, 총 %lu초(+휴식 %lu초). 단위는 raw LSB.\r\n",
         (unsigned long)COLLECT_FS_HZ, (unsigned long)SESS_N,
         (unsigned long)total, (unsigned long)(COLLECT_REST_S * (SESS_N - 1u)));
  printf("LD2 켜짐=흔드세요 / 꺼짐=손대지 마세요 / 점멸=준비\r\n");

  for (uint32_t i = 0u; i < SESS_N; i++)
  {
    printf("\r\n[COLLECT] --- 세션 %lu/%lu : %s (%lu초) ---\r\n",
           (unsigned long)(i + 1u), (unsigned long)SESS_N,
           k_sess[i].label, (unsigned long)k_sess[i].secs);
    prv_rest(COLLECT_REST_S, k_sess[i].label, k_sess[i].hint);
    prv_session(hi2c, i);
  }

  printf("\r\n=== 수집 완료: 세션 %lu개 ===\r\n", (unsigned long)SESS_N);
  printf("PC에서: python3 model/import_board_csv.py <이 로그파일>\r\n");

  /* 여기서 멈춘다. 스케줄러로 넘어가면 로그 태스크가 UART를 같이 쓰게 되고,
   * 사용자가 로그를 저장하기도 전에 화면이 [EDGE] 줄로 덮인다.
   * LD2를 2초 주기로 느리게 점멸시켜 "정상 종료"임을 알린다 -
   * 스택 오버플로 훅(1초 주기)과 구분되는 패턴이다. */
  for (;;)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
  }
}
