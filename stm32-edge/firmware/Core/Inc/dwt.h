/**
  ******************************************************************************
  * @file    dwt.h
  * @brief   DWT 사이클 카운터 + 통계 + CSV 덤프 (이 프로젝트의 측정 기반)
  *
  * 측정 규칙(CLAUDE.md): 반복 >= 100, 워밍업 제외, 중앙값/p99 병기.
  * 단발 측정치는 신뢰하지 않는다.
  *
  * 클럭 주의: 84MHz는 HSI(내부 RC, +-1%) 기반이라 절대시간 정밀도가 없다.
  * 따라서 기록/비교의 원본 단위는 항상 "사이클"이고, us/ms는 84MHz 기준 환산치다.
  ******************************************************************************
  */
#ifndef __DWT_H
#define __DWT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ 카운터 */

/**
  * @brief  DWT 사이클 카운터를 켠다. 다른 dwt_* 호출 전에 한 번만.
  * @retval true  = 사용 가능
  *         false = 이 코어에 사이클 카운터가 없음(DWT_CTRL의 NOCYCCNT)
  */
bool dwt_init(void);

/** @brief 현재 사이클 카운터 값. */
static inline uint32_t dwt_now(void)
{
  return DWT->CYCCNT;
}

/**
  * @brief  c0 이후 경과 사이클.
  * @note   32비트 unsigned 뺄셈은 2^32 모듈로 연산이라 랩어라운드 1회까지는
  *         자동으로 맞는다. 단 84MHz에서 2^32 사이클 = 약 51초이므로
  *         측정 구간이 51초를 넘으면 안 되고, 중간에 int로 캐스팅하면 깨진다.
  */
static inline uint32_t dwt_elapsed(uint32_t c0)
{
  return DWT->CYCCNT - c0;
}

/* ------------------------------------------------------------------ 통계 */

typedef struct
{
  uint32_t n;
  uint32_t min;
  uint32_t med;   /* 중앙값 */
  uint32_t p99;   /* nearest-rank */
  uint32_t max;   /* 실시간 시스템에서는 이게 진짜 지표(WCET 관점) */
} dwt_stats_t;

/**
  * @brief  buf에서 min/중앙값/p99/max를 뽑는다.
  * @note   buf를 제자리 정렬한다(파괴적). 원본이 필요하면 먼저 덤프할 것.
  *         malloc 없음 - 삽입정렬 O(n^2), n=100이면 충분히 싸다.
  */
void dwt_stats(uint32_t *buf, uint32_t n, dwt_stats_t *out);

/* ------------------------------------------------------------------ 출력 */

/** @brief 요약 CSV 헤더 한 줄. */
void dwt_csv_header(void);

/** @brief 요약 CSV 한 줄. measurements 폴더의 csv에 그대로 붙여넣을 수 있는 형식. */
void dwt_csv_row(const char *tag, const dwt_stats_t *s);

/**
  * @brief  원본 사이클 전량을 CSV로 덤프 (히스토그램용).
  * @note   측정 중에 printf하면 probe effect가 생긴다. 반드시 측정을 다 끝내고
  *         RAM 버퍼에 모은 뒤 한꺼번에 부를 것.
  */
void dwt_csv_dump(const char *tag, const uint32_t *buf, uint32_t n);

/* ------------------------------------------------------------------ 자가검증 */

/**
  * @brief  DWT가 실제로 신뢰할 만한지 3가지로 검증하고 결과를 UART에 찍는다.
  *         [1] 살아있나  [2] 눈금이 맞나  [3] 측정 오버헤드는 얼마인가
  * @note   스케줄러 시작 전에 부를 것. 태스크가 없어야 순수한 바닥값이 나온다.
  * @retval true = 3가지 모두 통과
  */
bool dwt_selftest(void);

/**
  * @brief  측정 오버헤드(빈 구간의 중앙값 사이클). dwt_selftest()가 채운다.
  * @note   짧은 구간을 잴 때는 이 값을 빼거나 최소한 같이 보고할 것.
  */
extern uint32_t dwt_overhead_cycles;

#ifdef __cplusplus
}
#endif

#endif /* __DWT_H */