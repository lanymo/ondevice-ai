/**
  ******************************************************************************
  * @file    dwt.c
  * @brief   DWT 사이클 카운터 + 통계 + CSV 덤프
  ******************************************************************************
  */
#include "dwt.h"
#include <stdio.h>

/* 자가검증 반복 횟수 (CLAUDE.md: 반복 >= 100, 워밍업 제외) */
#define DWT_ST_WARMUP   10u
#define DWT_ST_N        100u

uint32_t dwt_overhead_cycles = 0u;

/* 정적 버퍼만 - malloc 금지 (100 * 4B = 400B) */
static uint32_t s_buf[DWT_ST_N];

/* ------------------------------------------------------------------ 카운터 */

bool dwt_init(void)
{
  /* 1) 트레이스 유닛 인에이블. 이걸 먼저 켜야 DWT 레지스터가 유효하다. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* 이 코어에 사이클 카운터가 있는지 확인 (없으면 NOCYCCNT가 1) */
  if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0u)
  {
    return false;
  }

  /* 2) 카운터 0으로  3) 카운터 인에이블 */
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  return true;
}

/* ------------------------------------------------------------------ 통계 */

static void dwt_sort(uint32_t *a, uint32_t n)
{
  for (uint32_t i = 1u; i < n; ++i)
  {
    uint32_t key = a[i];
    uint32_t j = i;
    while ((j > 0u) && (a[j - 1u] > key))
    {
      a[j] = a[j - 1u];
      --j;
    }
    a[j] = key;
  }
}

void dwt_stats(uint32_t *buf, uint32_t n, dwt_stats_t *out)
{
  if ((out == NULL) || (buf == NULL) || (n == 0u))
  {
    if (out != NULL)
    {
      out->n = 0u; out->min = 0u; out->med = 0u; out->p99 = 0u; out->max = 0u;
    }
    return;
  }

  dwt_sort(buf, n);

  out->n   = n;
  out->min = buf[0];
  out->max = buf[n - 1u];

  /* 중앙값: 짝수면 가운데 둘의 평균 */
  if ((n & 1u) != 0u)
  {
    out->med = buf[n / 2u];
  }
  else
  {
    out->med = (uint32_t)(((uint64_t)buf[(n / 2u) - 1u] + (uint64_t)buf[n / 2u]) / 2u);
  }

  /* p99: nearest-rank. idx = ceil(n * 99 / 100), 1-based */
  {
    uint32_t idx = (uint32_t)((((uint64_t)n * 99u) + 99u) / 100u);
    if (idx == 0u)
    {
      idx = 1u;
    }
    out->p99 = buf[idx - 1u];
  }
}

/* ------------------------------------------------------------------ 출력 */

void dwt_csv_header(void)
{
  printf("tag,n,min,median,p99,max\r\n");
}

void dwt_csv_row(const char *tag, const dwt_stats_t *s)
{
  printf("%s,%lu,%lu,%lu,%lu,%lu\r\n",
         tag,
         (unsigned long)s->n,
         (unsigned long)s->min,
         (unsigned long)s->med,
         (unsigned long)s->p99,
         (unsigned long)s->max);
}

void dwt_csv_dump(const char *tag, const uint32_t *buf, uint32_t n)
{
  printf("tag,i,cycles\r\n");
  for (uint32_t i = 0u; i < n; ++i)
  {
    printf("%s,%lu,%lu\r\n", tag, (unsigned long)i, (unsigned long)buf[i]);
  }
}

/* ------------------------------------------------------------------ 자가검증 */

/* [1] 카운터가 실제로 도는가 */
static bool dwt_st_alive(void)
{
  uint32_t a = dwt_now();
  for (volatile uint32_t i = 0u; i < 100u; ++i) { }  /* volatile: 최적화로 사라지는 것 방지 */
  uint32_t b = dwt_now();

  if (b == a)
  {
    printf("[1] alive    : FAIL  CYCCNT 고정 (TRCENA/CYCCNTENA 확인)\r\n");
    return false;
  }

  printf("[1] alive    : PASS  %lu -> %lu\r\n", (unsigned long)a, (unsigned long)b);
  return true;
}

/* [2] 눈금이 맞는가 - 알려진 지연과 대조 */
static bool dwt_st_scale(void)
{
  const uint32_t ms = 100u;

  /* 기대값을 상수로 박지 않는다. 클럭이 바뀌면 기대값도 따라 바뀌어야 한다. */
  const uint32_t expect = (SystemCoreClock / 1000u) * ms;

  uint32_t c0 = dwt_now();
  HAL_Delay(ms);
  uint32_t meas = dwt_elapsed(c0);

  /* 오차를 퍼밀(1/1000)로. 부동소수점 없이 */
  int32_t err = (int32_t)((((int64_t)meas - (int64_t)expect) * 1000) / (int64_t)expect);

  /* 허용 범위를 비대칭으로 두는 이유:
   * HAL_Delay는 "최소 ms 만큼"을 보장하려고 1틱을 더 기다린다. 즉 항상 조금 길다.
   * 짧게 나오는 건 이상하지만 5% 정도 길게 나오는 건 정상이다.
   * (HSI +-1%는 여기서 상쇄된다 - DWT와 TIM10이 같은 HSI에서 나오므로 비율은 유지된다) */
  bool ok = ((err >= -10) && (err <= 50));

  printf("[2] scale    : %s  expect=%lu meas=%lu err=%+ld permille\r\n",
         ok ? "PASS" : "FAIL",
         (unsigned long)expect,
         (unsigned long)meas,
         (long)err);

  if (!ok)
  {
    printf("             -> 크게 틀리면 클럭 가정(SystemCoreClock=%lu)이 틀린 것\r\n",
           (unsigned long)SystemCoreClock);
  }
  return ok;
}

/* [3] 측정 자체가 몇 사이클을 먹는가 (바닥값) */
static bool dwt_st_overhead(void)
{
  dwt_stats_t st;

  /* 워밍업 - 측정에서 제외 */
  for (uint32_t i = 0u; i < DWT_ST_WARMUP; ++i)
  {
    uint32_t t = dwt_now();
    (void)dwt_elapsed(t);
  }

  /* 빈 구간을 100회 */
  for (uint32_t i = 0u; i < DWT_ST_N; ++i)
  {
    uint32_t t = dwt_now();
    s_buf[i] = dwt_elapsed(t);
  }

  dwt_stats(s_buf, DWT_ST_N, &st);
  dwt_overhead_cycles = st.med;

  printf("[3] overhead : n=%lu min=%lu med=%lu p99=%lu max=%lu (cycles)\r\n",
         (unsigned long)st.n,
         (unsigned long)st.min,
         (unsigned long)st.med,
         (unsigned long)st.p99,
         (unsigned long)st.max);

  /* 수~수십 사이클이면 정상. 수백이면 뭔가 끼어들었다는 뜻 */
  if (st.med > 200u)
  {
    printf("             -> 오버헤드가 비정상적으로 크다\r\n");
    return false;
  }
  return true;
}

bool dwt_selftest(void)
{
  bool ok = true;

  printf("\r\n--- DWT selftest ---\r\n");

  if (!dwt_st_alive())
  {
    printf("--- DWT selftest: FAIL (이후 측정 신뢰 불가) ---\r\n");
    return false;  /* 안 도는 카운터로 나머지를 재는 건 의미 없다 */
  }

  ok = dwt_st_scale()    && ok;
  ok = dwt_st_overhead() && ok;

  printf("--- DWT selftest: %s ---\r\n", ok ? "PASS" : "FAIL");

  /* 측정 파이프라인(통계 -> CSV)이 실제로 도는지도 여기서 같이 확인해둔다.
   * 4번 항목(액션 지연)이 이 두 함수를 그대로 쓴다. */
  dwt_csv_header();
  {
    dwt_stats_t st;
    dwt_stats(s_buf, DWT_ST_N, &st);   /* 이미 정렬돼 있음 - 재정렬해도 무해 */
    dwt_csv_row("dwt_overhead", &st);
  }

  return ok;
}