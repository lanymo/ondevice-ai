/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU-6050 (GY-521) 브링업 드라이버
  ******************************************************************************
  */
#include "mpu6050.h"
#include "dwt.h"
#include <stdio.h>

/* HAL은 8비트로 좌시프트된 주소를 받는다 (7비트 << 1) */
#define MPU_HAL_ADDR   ((uint16_t)(MPU6050_ADDR_7B << 1))
#define MPU_TIMEOUT    100u   /* ms */

/* 100Hz 스트림 검증 파라미터 (CLAUDE.md: 반복 >= 100, 워밍업 제외) */
#define STREAM_WARMUP  10u
#define STREAM_N       100u
#define STREAM_PERIOD_MS 10u  /* 10ms = 100Hz 목표 */

/* 정적 버퍼만 - malloc 금지. 400B x 2 */
static uint32_t s_interval[STREAM_N];   /* 샘플 간격 (사이클) */
static uint32_t s_readcost[STREAM_N];   /* 읽기 1회 소요 (사이클) */

/* ------------------------------------------------------------------ 기본 */

/* GY-521 이름으로 유통되는 모듈의 실제 다이는 제각각이다.
 * 아래 셋은 우리가 쓰는 레지스터(PWR_MGMT_1, ACCEL_XOUT_H)가 동일하다. */
static const struct
{
  uint8_t     id;
  const char *name;
} k_known_ids[] = {
  { 0x68u, "MPU-6050" },
  { 0x70u, "MPU-6500" },
  { 0x71u, "MPU-9250" },
};

const char *mpu6050_id_name(uint8_t who)
{
  for (uint32_t i = 0u; i < (sizeof(k_known_ids) / sizeof(k_known_ids[0])); ++i)
  {
    if (k_known_ids[i].id == who)
    {
      return k_known_ids[i].name;
    }
  }
  return NULL;
}

bool mpu6050_who_am_i(I2C_HandleTypeDef *hi2c, uint8_t *out_val)
{
  uint8_t v = 0u;

  if (HAL_I2C_Mem_Read(hi2c, MPU_HAL_ADDR, MPU6050_REG_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &v, 1u, MPU_TIMEOUT) != HAL_OK)
  {
    return false;
  }

  if (out_val != NULL)
  {
    *out_val = v;
  }
  return true;   /* 읽기 성공 여부만. 값 판정은 호출자가 한다 */
}

bool mpu6050_wake(I2C_HandleTypeDef *hi2c)
{
  uint8_t v = 0u;

  if (HAL_I2C_Mem_Read(hi2c, MPU_HAL_ADDR, MPU6050_REG_PWR_MGMT_1,
                       I2C_MEMADD_SIZE_8BIT, &v, 1u, MPU_TIMEOUT) != HAL_OK)
  {
    return false;
  }

  v &= (uint8_t)~MPU6050_PWR1_SLEEP_Msk;   /* SLEEP 비트만 지운다 */

  if (HAL_I2C_Mem_Write(hi2c, MPU_HAL_ADDR, MPU6050_REG_PWR_MGMT_1,
                        I2C_MEMADD_SIZE_8BIT, &v, 1u, MPU_TIMEOUT) != HAL_OK)
  {
    return false;
  }

  /* 써놓고 끝내지 않는다. 실제로 지워졌는지 읽어서 확인 */
  if (HAL_I2C_Mem_Read(hi2c, MPU_HAL_ADDR, MPU6050_REG_PWR_MGMT_1,
                       I2C_MEMADD_SIZE_8BIT, &v, 1u, MPU_TIMEOUT) != HAL_OK)
  {
    return false;
  }

  return ((v & MPU6050_PWR1_SLEEP_Msk) == 0u);
}

/* ------------------------------------------------------------------ 설정 */

/* DLPF_CFG -> 가속도 대역폭(Hz). RM-MPU-6000A의 표를 옮긴 것 - 확인 대상이다
 * (docs/references.md §3). 자이로 대역폭은 몇 Hz씩 다르지만(256/188/98/42/20/10/5)
 * 우리 판단 기준은 "나이퀴스트 50Hz보다 좁은가"라 더 넓은 쪽인 가속도만 본다. */
static const uint16_t k_dlpf_accel_bw[8] = { 260u, 184u, 94u, 44u, 21u, 10u, 5u, 0u };

uint16_t mpu6050_dlpf_accel_bw_hz(uint8_t dlpf_cfg)
{
  return k_dlpf_accel_bw[dlpf_cfg & MPU6050_DLPF_CFG_Msk];
}

/** 레지스터 하나를 read-modify-write. 마스크 밖 비트는 보존한다. */
static bool prv_rmw(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t msk, uint8_t val)
{
  uint8_t v = 0u;

  if (HAL_I2C_Mem_Read(hi2c, MPU_HAL_ADDR, reg,
                       I2C_MEMADD_SIZE_8BIT, &v, 1u, MPU_TIMEOUT) != HAL_OK)
  {
    return false;
  }

  v = (uint8_t)((v & (uint8_t)~msk) | (val & msk));

  return (HAL_I2C_Mem_Write(hi2c, MPU_HAL_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, &v, 1u, MPU_TIMEOUT) == HAL_OK);
}

bool mpu6050_configure(I2C_HandleTypeDef *hi2c, mpu6050_cfg_t *out)
{
  mpu6050_cfg_t rb = { 0u, 0u, 0u, 0u };
  const uint8_t want_accel = (uint8_t)(MPU6050_AFS_SEL << MPU6050_AFS_SEL_Pos);
  const uint8_t want_gyro  = (uint8_t)(MPU6050_FS_SEL  << MPU6050_FS_SEL_Pos);
  const uint8_t want_dlpf  = (uint8_t)MPU6050_DLPF_CFG;

  if (!prv_rmw(hi2c, MPU6050_REG_CONFIG,       MPU6050_DLPF_CFG_Msk, want_dlpf)  ||
      !prv_rmw(hi2c, MPU6050_REG_GYRO_CONFIG,  MPU6050_FS_SEL_Msk,   want_gyro)  ||
      !prv_rmw(hi2c, MPU6050_REG_ACCEL_CONFIG, MPU6050_AFS_SEL_Msk,  want_accel))
  {
    return false;
  }

  /* 되읽기. 0x19~0x1C가 연속이라 한 트랜잭션으로 받는다.
   * 구조체에 직접 받지 않는 건 uint8_t 4개라도 패딩 없음이 표준 보장은 아니라서다. */
  {
    uint8_t b[4];

    if (HAL_I2C_Mem_Read(hi2c, MPU_HAL_ADDR, MPU6050_REG_SMPLRT_DIV,
                         I2C_MEMADD_SIZE_8BIT, b, 4u, MPU_TIMEOUT) != HAL_OK)
    {
      return false;
    }
    rb.smplrt_div = b[0];
    rb.config     = b[1];
    rb.gyro_cfg   = b[2];
    rb.accel_cfg  = b[3];
  }

  if (out != NULL)
  {
    *out = rb;
  }

  /* 우리가 건드린 필드만 판정한다. 자기진단/EXT_SYNC 비트는 보존 대상이지
   * 검사 대상이 아니다. SMPLRT_DIV는 안 쓴 값이라 기대치(리셋 기본 0)로 확인만. */
  return ((rb.config    & MPU6050_DLPF_CFG_Msk) == want_dlpf)  &&
         ((rb.gyro_cfg  & MPU6050_FS_SEL_Msk)   == want_gyro)  &&
         ((rb.accel_cfg & MPU6050_AFS_SEL_Msk)  == want_accel) &&
         (rb.smplrt_div == 0u);
}

/* I2C1 배선 (docs/pinmap.md). 버스를 손으로 흔들 때만 쓴다. */
#define I2C_SCL_PIN   GPIO_PIN_8
#define I2C_SDA_PIN   GPIO_PIN_9
#define I2C_GPIO      GPIOB

/* HAL_Delay는 1ms 해상도라 100kHz 버스에 너무 굵다. DWT로 us를 만든다. */
static void prv_delay_us(uint32_t us)
{
  const uint32_t t0 = dwt_now();
  const uint32_t n = (SystemCoreClock / 1000000u) * us;

  while (dwt_elapsed(t0) < n)
  {
  }
}

bool mpu6050_recover(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef g = { 0 };

  /* MspDeInit이 PB8/PB9를 리셋 상태로 되돌린다 */
  (void)HAL_I2C_DeInit(hi2c);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Pin   = I2C_SCL_PIN | I2C_SDA_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_OD;   /* 오픈드레인 - 풀업은 GY-521 모듈에 있다 */
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(I2C_GPIO, &g);
  HAL_GPIO_WritePin(I2C_GPIO, I2C_SCL_PIN | I2C_SDA_PIN, GPIO_PIN_SET);
  prv_delay_us(10u);

  /* SCL을 최대 9번 친다. 슬레이브가 붙들고 있던 바이트를 다 밀어내면 SDA가 풀린다.
   * 9인 이유: 한 바이트 8비트 + ACK 1비트가 슬레이브가 붙들 수 있는 최대 길이다. */
  for (uint32_t i = 0u; i < 9u; i++)
  {
    if (HAL_GPIO_ReadPin(I2C_GPIO, I2C_SDA_PIN) == GPIO_PIN_SET)
    {
      break;                       /* 이미 풀렸다 */
    }
    HAL_GPIO_WritePin(I2C_GPIO, I2C_SCL_PIN, GPIO_PIN_RESET);
    prv_delay_us(5u);
    HAL_GPIO_WritePin(I2C_GPIO, I2C_SCL_PIN, GPIO_PIN_SET);
    prv_delay_us(5u);
  }

  /* STOP 조건을 손으로 만든다: SCL이 높은 동안 SDA를 낮->높.
   * 이걸 안 해주면 슬레이브는 아직 전송이 끝나지 않았다고 생각한다. */
  HAL_GPIO_WritePin(I2C_GPIO, I2C_SDA_PIN, GPIO_PIN_RESET);
  prv_delay_us(5u);
  HAL_GPIO_WritePin(I2C_GPIO, I2C_SCL_PIN, GPIO_PIN_SET);
  prv_delay_us(5u);
  HAL_GPIO_WritePin(I2C_GPIO, I2C_SDA_PIN, GPIO_PIN_SET);
  prv_delay_us(10u);

  HAL_GPIO_DeInit(I2C_GPIO, I2C_SCL_PIN | I2C_SDA_PIN);

  /* hi2c의 Init 구조체는 DeInit 후에도 남아 있으므로 그대로 다시 올린다.
   * MspInit이 핀을 AF4 오픈드레인으로 되돌린다. */
  if (HAL_I2C_Init(hi2c) != HAL_OK)
  {
    return false;
  }

  /* 센서 쪽 상태도 되돌린다. 되읽기까지 하는 두 함수라 여기가 복구 성공 판정이다. */
  return mpu6050_wake(hi2c) && mpu6050_configure(hi2c, NULL);
}

bool mpu6050_accel_saturated(const mpu6050_imu_t *s)
{
  return (s->ax >= MPU6050_SAT_LSB) || (s->ax <= -MPU6050_SAT_LSB) ||
         (s->ay >= MPU6050_SAT_LSB) || (s->ay <= -MPU6050_SAT_LSB) ||
         (s->az >= MPU6050_SAT_LSB) || (s->az <= -MPU6050_SAT_LSB);
}

bool mpu6050_gyro_saturated(const mpu6050_imu_t *s)
{
  return (s->gx >= MPU6050_SAT_LSB) || (s->gx <= -MPU6050_SAT_LSB) ||
         (s->gy >= MPU6050_SAT_LSB) || (s->gy <= -MPU6050_SAT_LSB) ||
         (s->gz >= MPU6050_SAT_LSB) || (s->gz <= -MPU6050_SAT_LSB);
}

bool mpu6050_read_accel(I2C_HandleTypeDef *hi2c, mpu6050_accel_t *out)
{
  uint8_t b[6];

  /* 6바이트 연속 읽기. 한 트랜잭션으로 받아야 3축이 같은 시점의 값이 된다.
   * 축마다 따로 읽으면 그 사이에 센서가 갱신돼 서로 다른 시점이 섞인다. */
  if (HAL_I2C_Mem_Read(hi2c, MPU_HAL_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                       I2C_MEMADD_SIZE_8BIT, b, 6u, MPU_TIMEOUT) != HAL_OK)
  {
    return false;
  }

  /* 빅엔디안 16비트 2의 보수 */
  out->x = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  out->y = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
  out->z = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
  return true;
}

bool mpu6050_read_imu(I2C_HandleTypeDef *hi2c, mpu6050_imu_t *out)
{
  uint8_t b[MPU6050_BURST_LEN];

  /* 가속도/온도/자이로를 한 트랜잭션으로. 나눠 읽으면 축 사이에 센서가 갱신돼
   * 서로 다른 시점의 값이 한 샘플에 섞인다. */
  if (HAL_I2C_Mem_Read(hi2c, MPU_HAL_ADDR, MPU6050_REG_BURST_START,
                       I2C_MEMADD_SIZE_8BIT, b, MPU6050_BURST_LEN,
                       MPU_TIMEOUT) != HAL_OK)
  {
    return false;
  }

  /* 빅엔디안 16비트 2의 보수. b[6..7]은 온도 - 안 쓰고 건너뛴다 */
  out->ax = (int16_t)(((uint16_t)b[0]  << 8) | b[1]);
  out->ay = (int16_t)(((uint16_t)b[2]  << 8) | b[3]);
  out->az = (int16_t)(((uint16_t)b[4]  << 8) | b[5]);
  out->gx = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
  out->gy = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
  out->gz = (int16_t)(((uint16_t)b[12] << 8) | b[13]);
  return true;
}

/* ------------------------------------------------------------------ 브링업 */

bool mpu6050_bringup(I2C_HandleTypeDef *hi2c)
{
  uint8_t who = 0u;
  mpu6050_accel_t a;

  printf("\r\n--- MPU6050 bringup ---\r\n");

  /* [2] WHO_AM_I - 통신 확인용. 값 자체로 실패시키지 않는다. */
  if (!mpu6050_who_am_i(hi2c, &who))
  {
    printf("[2] who_am_i : FAIL  I2C 읽기 실패\r\n");
    return false;
  }
  {
    const char *name = mpu6050_id_name(who);
    if (name != NULL)
    {
      printf("[2] who_am_i : PASS  0x%02X (%s)\r\n", (unsigned int)who, name);
    }
    else
    {
      printf("[2] who_am_i : WARN  0x%02X (미등록 ID - 계속 진행)\r\n",
             (unsigned int)who);
    }
  }

  /* [3] sleep 해제 */
  if (!mpu6050_wake(hi2c))
  {
    printf("[3] wake     : FAIL  SLEEP 비트가 안 지워짐\r\n");
    return false;
  }
  printf("[3] wake     : PASS  SLEEP cleared\r\n");

  HAL_Delay(50);   /* 깨어난 뒤 첫 변환까지 여유 */

  /* [3b] 풀스케일 레인지 + DLPF - 쓰고 되읽어 확인한다.
   * W2까지는 "리셋 기본값이겠지"라고 가정만 했고 확인한 적이 없었다.
   * 여기서 실패하면 레지스터 주소/마스크가 틀린 것이다(mpu6050.h 주석 참조). */
  {
    mpu6050_cfg_t cfg;
    const bool ok = mpu6050_configure(hi2c, &cfg);
    const uint16_t bw = mpu6050_dlpf_accel_bw_hz(cfg.config);

    printf("[3b] config  : %s  SMPLRT_DIV=0x%02X CONFIG=0x%02X "
           "GYRO_CFG=0x%02X ACCEL_CFG=0x%02X\r\n",
           ok ? "PASS" : "FAIL",
           (unsigned int)cfg.smplrt_div, (unsigned int)cfg.config,
           (unsigned int)cfg.gyro_cfg, (unsigned int)cfg.accel_cfg);
    printf("     accel AFS_SEL=%u -> +-%ug (%d LSB/g), "
           "gyro FS_SEL=%u -> +-%udps\r\n",
           (unsigned int)MPU6050_AFS_SEL, (unsigned int)(2u << MPU6050_AFS_SEL),
           MPU6050_ACCEL_LSB_PER_G,
           (unsigned int)MPU6050_FS_SEL, (unsigned int)(250u << MPU6050_FS_SEL));
    printf("     DLPF_CFG=%u -> accel BW %uHz (나이퀴스트 %uHz 대비 %s)\r\n",
           (unsigned int)MPU6050_DLPF_CFG, (unsigned int)bw,
           (unsigned int)(1000u / STREAM_PERIOD_MS / 2u),
           (bw <= (1000u / STREAM_PERIOD_MS / 2u)) ? "좁음(OK)"
                                                   : "**넓음 - 에일리어싱**");
    if (!ok)
    {
      return false;
    }
  }

  /* [4] 값이 실제로 들어오나 + 움직이면 변하나 (2초간 20샘플) */
  printf("[4] accel    : 2초간 20샘플 - 지금 보드를 기울여 보세요\r\n");
  printf("     i,ax,ay,az\r\n");
  for (uint32_t i = 0u; i < 20u; ++i)
  {
    if (!mpu6050_read_accel(hi2c, &a))
    {
      printf("     읽기 실패 @ %lu\r\n", (unsigned long)i);
      return false;
    }
    printf("     %lu,%d,%d,%d\r\n", (unsigned long)i, a.x, a.y, a.z);
    HAL_Delay(100);
  }
  printf("     (참고: 설정된 +-%ug에서 1g = %d LSB, 포화선 %d)\r\n",
         (unsigned int)(2u << MPU6050_AFS_SEL), MPU6050_ACCEL_LSB_PER_G,
         MPU6050_SAT_LSB);

  /* [5] 100Hz 스트림 - "100Hz다"라고 주장하지 말고 잰다 */
  {
    dwt_stats_t si, sr;
    uint32_t prev;

    for (uint32_t i = 0u; i < STREAM_WARMUP; ++i)   /* 워밍업은 통계에서 제외 */
    {
      (void)mpu6050_read_accel(hi2c, &a);
      HAL_Delay(STREAM_PERIOD_MS);
    }

    /* 기준 샘플을 한 번 먼저 돌린다. 이게 없으면 첫 구간이 "루프 진입 직전 ->
     * 첫 샘플"이 되어 의미 없는 값(수십 사이클)이 min을 오염시킨다. */
    prev = dwt_now();
    (void)mpu6050_read_accel(hi2c, &a);
    HAL_Delay(STREAM_PERIOD_MS);

    for (uint32_t i = 0u; i < STREAM_N; ++i)
    {
      uint32_t t0 = dwt_now();
      s_interval[i] = t0 - prev;    /* 직전 샘플 시작 -> 이번 샘플 시작 */
      prev = t0;

      if (!mpu6050_read_accel(hi2c, &a))
      {
        printf("[5] stream   : FAIL  읽기 실패 @ %lu\r\n", (unsigned long)i);
        return false;
      }
      s_readcost[i] = dwt_elapsed(t0);
      HAL_Delay(STREAM_PERIOD_MS);
    }

    dwt_stats(s_interval, STREAM_N, &si);
    dwt_stats(s_readcost, STREAM_N, &sr);

    printf("[5] stream   : n=%lu (target %luHz)\r\n",
           (unsigned long)STREAM_N, (unsigned long)(1000u / STREAM_PERIOD_MS));
    printf("     interval cycles  med=%lu p99=%lu max=%lu  -> %lu.%02lu ms (med)\r\n",
           (unsigned long)si.med, (unsigned long)si.p99, (unsigned long)si.max,
           (unsigned long)(si.med / (SystemCoreClock / 1000u)),
           (unsigned long)((si.med % (SystemCoreClock / 1000u)) * 100u / (SystemCoreClock / 1000u)));
    printf("     read cost cycles med=%lu p99=%lu max=%lu\r\n",
           (unsigned long)sr.med, (unsigned long)sr.p99, (unsigned long)sr.max);

    dwt_csv_header();
    dwt_csv_row("imu_interval", &si);
    dwt_csv_row("imu_read_cost", &sr);
  }

  printf("--- MPU6050 bringup: PASS ---\r\n");
  return true;
}