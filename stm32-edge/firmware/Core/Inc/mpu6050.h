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

/* 0x3B부터 14바이트를 한 번에 읽으면 아래 순서로 이어진다 (Register Map):
 *   0x3B..0x40 ACCEL_X/Y/Z, 0x41..0x42 TEMP, 0x43..0x48 GYRO_X/Y/Z
 * 온도는 안 쓰지만 중간에 끼어 있어 건너뛸 수 없다 - 두 번 나눠 읽는 것보다
 * 한 트랜잭션이 낫다(축 간 시점 일치 + I2C 오버헤드 절약). */
#define MPU6050_REG_BURST_START  MPU6050_REG_ACCEL_XOUT_H
#define MPU6050_BURST_LEN        14u

/* ---- 설정 레지스터 (W3에서 추가) ------------------------------------------
 *
 * W2까지 이 셋을 **한 번도 안 건드렸다.** "리셋 기본값이 +-2g일 것"이라고 가정만
 * 하고 LSB 상수를 하드코딩해뒀는데, 가정과 확인은 다르다. W3에서 (1) 명시적으로
 * 쓰고 (2) 되읽어 확인하고 (3) 실제로 포화하는지 카운터로 센다.
 *
 * ⚠️ 아래 주소/비트 위치는 **RM-MPU-6000A-00 (Register Map)로 확인할 것**
 *    (docs/references.md §3). 다만 mpu6050_configure()가 쓴 뒤 되읽어 비교하므로,
 *    주소나 마스크가 틀리면 부팅 시 UART에 FAIL로 즉시 드러난다 - WHO_AM_I가
 *    자기검증 역할을 하는 것과 같은 구조다.
 */
#define MPU6050_REG_SMPLRT_DIV   0x19u
#define MPU6050_REG_CONFIG       0x1Au   /* [2:0] DLPF_CFG, [5:3] EXT_SYNC_SET */
#define MPU6050_REG_GYRO_CONFIG  0x1Bu   /* [4:3] FS_SEL,  [7:5] 자기진단 */
#define MPU6050_REG_ACCEL_CONFIG 0x1Cu   /* [4:3] AFS_SEL, [7:5] 자기진단 */

#define MPU6050_DLPF_CFG_Msk     0x07u
#define MPU6050_FS_SEL_Pos       3u
#define MPU6050_FS_SEL_Msk       0x18u
#define MPU6050_AFS_SEL_Pos      3u
#define MPU6050_AFS_SEL_Msk      0x18u

/* ---- 이 프로젝트가 고른 값 --------------------------------------------------
 *
 * [AFS_SEL=0 → +-2g]  진동 예지보전에 +-2g가 맞는지는 **아직 데이터로 답 못 했다.**
 *   az에는 중력 1g가 이미 깔려 있어 위쪽 여유가 1g뿐이고, 손으로 흔드는 proxy는
 *   2g를 넘길 수 있다. 넘으면 값이 32767에 붙어 p2p/std 특징이 통째로 망가진다.
 *   그래서 값을 바꾸는 대신 **포화 카운터**(mpu6050_accel_saturated)를 달아
 *   실제로 붙는지 센다. 붙으면 그때 AFS_SEL=1(+-4g)로 올린다 -
 *   모델 정규화는 물리 단위(g) 기준이라 **레인지를 바꿔도 재학습이 필요 없다**
 *   (model/config.py). 즉 이 결정은 되돌리기 싸고, 지금 필요한 건 근거다.
 *   대가는 분해능: +-2g에서 1 LSB = 61 ug, +-4g면 122 ug. 학습셋 기준 가장 작은
 *   특징 산포가 az.std ~ 5.3e-3 g = 87 LSB라 4g로 가도 여유는 있다.
 *
 * [FS_SEL=0 → +-250dps]  같은 논리. 자이로도 포화 카운터로 관측한다.
 *
 * [DLPF_CFG=3 → 대역폭 ~44Hz]  이건 값을 **바꾼다.** 리셋 기본값(0)은 대역폭이
 *   260Hz인데 우리 폴링은 100Hz라 나이퀴스트가 50Hz다. 50~260Hz 성분이 사라지는
 *   게 아니라 0~50Hz로 **접혀 들어온다**(에일리어싱). 하필 우리 특징 4개 중
 *   mad(인접 차분)가 고주파 에너지 proxy라 가짜 고주파를 그대로 먹고, 접히는
 *   위치가 진동 주파수에 따라 비단조적으로 변해 **같은 동작이 재현되지 않는다.**
 *   대가는 군지연 ~4.9ms인데 윈도우가 1.28초라 0.4% - 사실상 공짜다.
 *
 * [SMPLRT_DIV=0]  건드리지 않는다. DLPF!=0이면 내부 출력레이트가 1kHz가 되는데,
 *   이미 44Hz로 대역제한된 신호를 우리가 100Hz로 데시메이션하는 것이라 여기서
 *   에일리어싱이 추가로 생기지 않는다. 반대로 내부 레이트를 100Hz로 "맞추면"
 *   두 100Hz 클럭(STM32 HSI +-1% vs MPU 내부 발진기)이 서로 드리프트해서
 *   같은 샘플을 두 번 읽거나 하나를 건너뛰는 맥놀이가 생긴다. DRDY 인터럽트
 *   없이 폴링하는 한 **내부 레이트는 넉넉히 높은 편이 옳다.**
 */
#define MPU6050_AFS_SEL          0u   /* 0=+-2g 1=+-4g 2=+-8g 3=+-16g */
#define MPU6050_FS_SEL           0u   /* 0=+-250 1=+-500 2=+-1000 3=+-2000 dps */
#define MPU6050_DLPF_CFG         3u   /* 0=260Hz(기본) 1=184 2=94 3=44 4=21 5=10 6=5 */

/* 위 선택에서 유도된다 - 상수를 손으로 안 고치게 해서 레인지와 환산이 갈라지는
 * 사고를 막는다. 가속도는 16384>>AFS_SEL, 자이로는 131.0/2^FS_SEL. */
#define MPU6050_ACCEL_LSB_PER_G  (16384 >> MPU6050_AFS_SEL)
#define MPU6050_GYRO_LSB_PER_DPS (131.0f / (float)(1u << MPU6050_FS_SEL))

/* 포화 판정선. int16 만적은 32767이고, 그 97.7%를 넘으면 클리핑 위험 구간으로 본다.
 * 32767에 정확히 붙은 것만 세면 "거의 붙은" 샘플을 놓쳐 늦게 안다. */
#define MPU6050_SAT_LSB          32000

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} mpu6050_accel_t;

/**
  * 6축 한 샘플. 모델(model/config.py)의 CHANNELS = (ax,ay,az,gx,gy,gz)와 순서를 맞춘다.
  * raw int16 그대로 보관 - g/dps 환산은 윈도우를 뽑는 시점에 한다(링버퍼 크기 절약).
  */
typedef struct
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} mpu6050_imu_t;

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

/** 되읽은 설정 레지스터 원본. 로깅/디버깅용 - 판정은 mpu6050_configure가 한다. */
typedef struct
{
  uint8_t smplrt_div;   /* 0x19 */
  uint8_t config;       /* 0x1A */
  uint8_t gyro_cfg;     /* 0x1B */
  uint8_t accel_cfg;    /* 0x1C */
} mpu6050_cfg_t;

/**
  * @brief  풀스케일 레인지(AFS_SEL/FS_SEL)와 DLPF를 위 상수대로 쓰고 **되읽어 확인**한다.
  * @param  out  되읽은 레지스터 4개 (NULL 가능)
  * @retval 네 레지스터가 전부 의도한 값으로 읽혔는가.
  * @note   read-modify-write다 - 자기진단 비트와 EXT_SYNC_SET은 건드리지 않는다.
  *         멱등이라 여러 번 불러도 무해하다(부팅 브링업 + 센서 태스크 양쪽에서 부른다).
  */
bool mpu6050_configure(I2C_HandleTypeDef *hi2c, mpu6050_cfg_t *out);

/** @brief DLPF_CFG 값에 대응하는 가속도 대역폭(Hz). 모르는 값이면 0. */
uint16_t mpu6050_dlpf_accel_bw_hz(uint8_t dlpf_cfg);

/**
  * @brief  I2C 버스를 강제로 풀고 센서를 재초기화한다. 읽기가 연속 실패할 때 부른다.
  * @retval 복구 성공 여부 (wake + configure까지 되읽기로 확인한다).
  *
  * @note   **왜 필요한가 (2026-08-08 실측)**: 수집 중 배선이 흔들려 통신이 끊기자
  *         세션 하나가 죽은 게 아니라 **그 뒤 세션이 전부 죽었다**(5,000/5,000 실패).
  *         리셋하면 멀쩡히 살아났으니 센서가 고장난 게 아니라 **버스가 잠긴** 것이다.
  *         전송 도중 마스터가 사라지면 슬레이브가 바이트를 내보내던 중이라 SDA를
  *         낮게 붙들고, 그 상태에선 START 조건 자체를 만들 수 없어 영원히 못 빠져나온다.
  *         HAL_I2C_Init만 다시 불러도 안 된다 - 잠근 건 슬레이브지 페리페럴이 아니다.
  *         그래서 핀을 GPIO로 되돌려 **SCL을 손으로 9번 쳐서** 슬레이브가 남은 비트를
  *         밀어내게 하고, STOP을 만들어준 뒤 페리페럴을 다시 올린다.
  * @note   블로킹이고 수 ms 걸린다. ISR에서 부르지 말 것.
  */
bool mpu6050_recover(I2C_HandleTypeDef *hi2c);

/** @brief 한 샘플의 가속도 3축 중 하나라도 포화 근처인가 (MPU6050_SAT_LSB). */
bool mpu6050_accel_saturated(const mpu6050_imu_t *s);

/** @brief 한 샘플의 자이로 3축 중 하나라도 포화 근처인가. */
bool mpu6050_gyro_saturated(const mpu6050_imu_t *s);

/**
  * @brief  PWR_MGMT_1의 SLEEP 비트를 지워 센서를 깨운다. 브링업 3단계.
  * @note   이걸 안 하면 통신은 되는데 가속도 값이 계속 0으로 나온다.
  *         "배선은 맞는데 값이 안 변한다"의 대부분이 이것.
  */
bool mpu6050_wake(I2C_HandleTypeDef *hi2c);

/** @brief 가속도 3축을 6바이트 연속 읽기로 가져온다. (브링업 경로용) */
bool mpu6050_read_accel(I2C_HandleTypeDef *hi2c, mpu6050_accel_t *out);

/**
  * @brief  가속도+자이로 6축을 14바이트 연속 읽기 한 번으로 가져온다. (추론 입력용)
  * @note   6바이트 읽기보다 I2C 점유가 약 2배다. 100kHz에서 대략 0.84ms -> 1.5ms.
  *         10ms 주기 안에서는 여유가 있지만, 주기를 더 줄이려면 400kHz 전환을 검토할 것.
  */
bool mpu6050_read_imu(I2C_HandleTypeDef *hi2c, mpu6050_imu_t *out);

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