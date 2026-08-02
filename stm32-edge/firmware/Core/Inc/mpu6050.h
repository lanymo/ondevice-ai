/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU-6050 (GY-521) 브링업 드라이버
  *
  * W2 DoD: WHO_AM_I 확인 -> sleep 해제 -> 가속도 읽기 -> 100Hz 스트림.
  *
  * 레지스터 주소/기대값은 MPU-6000/6050 Register Map 기준이며 아래 한곳에
  * 모아두었다. WHO_AM_I 검사가 사실상 자기검증 역할을 한다 - 주소가 틀리면
  * 기대값이 안 나온다.
  ******************************************************************************
  */
#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- I2C 주소 (7비트). AD0=GND -> 0x68, AD0=VCC -> 0x69 ---- */
#define MPU6050_ADDR_7B       0x68u

/* ---- 레지스터 (MPU-6000/6050 Register Map) ---- */
#define MPU6050_REG_WHO_AM_I     0x75u
#define MPU6050_WHO_AM_I_EXPECT  0x68u
#define MPU6050_REG_PWR_MGMT_1   0x6Bu
#define MPU6050_PWR1_SLEEP_Msk   0x40u   /* bit 6 */
#define MPU6050_REG_ACCEL_XOUT_H 0x3Bu   /* 여기부터 6바이트: XH XL YH YL ZH ZL */

/* 리셋 직후 가속도 풀스케일은 +-2g. 그때 1g = 16384 LSB */
#define MPU6050_ACCEL_LSB_PER_G  16384

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} mpu6050_accel_t;

/**
  * @brief  WHO_AM_I를 읽는다. 브링업 2단계.
  * @param  out_val 읽힌 값 (NULL 가능)
  * @retval I2C 읽기 성공 여부. **값이 기대치인지는 판단하지 않는다** -
  *         GY-521로 팔리는 모듈에 6050 대신 6500 계열 다이가 들어있는 경우가
  *         흔한데, 우리가 쓰는 레지스터는 동일해서 그대로 동작한다.
  *         실제 판정은 가속도 값이 움직임에 반응하는지로 한다.
  */
bool mpu6050_who_am_i(I2C_HandleTypeDef *hi2c, uint8_t *out_val);

/** @brief WHO_AM_I 값에 대응하는 칩 이름. 모르는 값이면 NULL. */
const char *mpu6050_id_name(uint8_t who);

/**
  * @brief  PWR_MGMT_1의 SLEEP 비트를 지워 센서를 깨운다. 브링업 3단계.
  * @note   이걸 안 하면 통신은 되는데 가속도 값이 계속 0으로 나온다.
  *         "배선은 맞는데 값이 안 변한다"의 대부분이 이것.
  */
bool mpu6050_wake(I2C_HandleTypeDef *hi2c);

/** @brief 가속도 3축을 6바이트 연속 읽기로 가져온다. */
bool mpu6050_read_accel(I2C_HandleTypeDef *hi2c, mpu6050_accel_t *out);

/**
  * @brief  브링업 전 과정을 순서대로 실행하고 결과를 UART에 찍는다.
  *         [2] WHO_AM_I  [3] sleep 해제  [4] 값 변화 확인  [5] 100Hz 스트림 검증
  * @note   스케줄러 시작 전에 부를 것 (블로킹 API 사용).
  */
bool mpu6050_bringup(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */