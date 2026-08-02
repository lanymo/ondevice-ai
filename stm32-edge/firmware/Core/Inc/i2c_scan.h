/**
  ******************************************************************************
  * @file    i2c_scan.h
  * @brief   I2C 버스 스캔 - 브링업 1단계 진단 도구
  *
  * 배선 문제와 소프트웨어 문제를 한 번에 가르기 위한 것.
  * 스캔에서 주소가 뜨면 전원/GND/SDA/SCL 4가닥이 전부 맞았다는 뜻이고,
  * 이후 문제는 전부 레지스터 쪽이다. 아무것도 안 뜨면 코드는 볼 필요가 없다.
  ******************************************************************************
  */
#ifndef __I2C_SCAN_H
#define __I2C_SCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/**
  * @brief  7비트 주소 공간(0x08~0x77)을 훑어 응답하는 장치를 UART에 출력한다.
  * @param  hi2c  초기화된 I2C 핸들 (예: &hi2c1)
  * @retval 응답한 장치 수
  * @note   스케줄러 시작 전에 부를 것. 블로킹 API를 쓴다.
  */
uint32_t i2c_bus_scan(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_SCAN_H */