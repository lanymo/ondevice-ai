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
  printf("     (참고: 기본 +-2g에서 1g = %d LSB)\r\n", MPU6050_ACCEL_LSB_PER_G);

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