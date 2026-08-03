/**
  ******************************************************************************
  * @file    i2c_scan.c
  * @brief   I2C 버스 스캔
  ******************************************************************************
  */
#include "i2c_scan.h"
#include <stdio.h>

/* 7비트 주소의 유효 범위. 0x00~0x07과 0x78~0x7F는 예약 주소라 건너뛴다. */
#define I2C_ADDR_FIRST   0x08u
#define I2C_ADDR_LAST    0x77u

#define I2C_SCAN_TRIALS  2u
#define I2C_SCAN_TIMEOUT 5u   /* ms. 전부 무응답이어도 최악 약 1.1초 */

uint32_t i2c_bus_scan(I2C_HandleTypeDef *hi2c)
{
  uint32_t found = 0u;

  printf("\r\n--- I2C bus scan ---\r\n");

  for (uint8_t addr = I2C_ADDR_FIRST; addr <= I2C_ADDR_LAST; ++addr)
  {
    /* HAL은 8비트로 좌시프트된 주소를 받는다. 7비트 주소를 그냥 넘기면
     * 엉뚱한 장치를 부르게 되고 영원히 아무것도 안 뜬다.
     * (docs/parts-guide.md §2에 "첫 삽질 포인트"로 적혀 있는 그것) */
    uint16_t hal_addr = (uint16_t)((uint16_t)addr << 1);

    if (HAL_I2C_IsDeviceReady(hi2c, hal_addr, I2C_SCAN_TRIALS, I2C_SCAN_TIMEOUT) == HAL_OK)
    {
      printf("  found 0x%02X\r\n", (unsigned int)addr);
      ++found;
    }
  }

  if (found == 0u)
  {
    printf("  (없음)\r\n");
    printf("  -> 배선(SDA/SCL 스왑), GND 공통, VCC 3.3V, 풀업 순으로 확인\r\n");
  }

  printf("--- scan done: %lu device(s) ---\r\n", (unsigned long)found);
  return found;
}