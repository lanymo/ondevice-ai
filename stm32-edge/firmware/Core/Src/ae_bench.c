/**
  ******************************************************************************
  * @file    ae_bench.c
  * @brief   추론 자가검증 + float32/int8 커널 사이클 (부팅 시 1회, 스케줄러 전)
  *
  * 측정 규칙(CLAUDE.md): 반복 >= 100, 워밍업 제외, 중앙값/p99/max 병기.
  * 메모리 규칙: malloc 금지 - 전부 파일 스코프 static.
  ******************************************************************************
  */
#include "ae_bench.h"

#include "ae_infer.h"
#include "ae_infer_f32.h"
#include "ae_testvec_board.h"
#include "dwt.h"

#include <math.h>
#include <stdio.h>

/* 반복 수. 워밍업을 뺀 뒤에도 200개면 p99가 nearest-rank로 의미를 갖는다
 * (200개의 p99 = 위에서 두 번째. 100개면 p99가 곧 max라 두 지표가 겹친다). */
#define BENCH_N       200u
#define BENCH_WARMUP   16u

/* 사이클 버퍼 4종 x 200 x 4B = 3,200 B. 스택이 아니라 static인 이유는
 * 이 함수가 main()에서 불려 MSP 스택을 쓰기 때문 - 3KB면 기본 스택을 넘긴다. */
static uint32_t s_cyc_feat[BENCH_N];
static uint32_t s_cyc_quant[BENCH_N];
static uint32_t s_cyc_i8[BENCH_N];
static uint32_t s_cyc_f32[BENCH_N];

/* 추론 중간 결과. ae_run_window를 안 쓰고 단계를 풀어 쓰는 건 단계별로
 * 사이클을 나눠 재기 위해서다 - 합쳐 재면 공통 비용(특징 추출)이 두 커널
 * 숫자에 다 섞여 정작 보려는 차이가 희석된다 (system-design.md §5). */
static float  s_feat[AE_N_FEATURES];
static int8_t s_q_in[AE_N_FEATURES];
static int8_t s_q_out[AE_N_FEATURES];
static float  s_recon[AE_N_FEATURES];

/* float 기대값과의 허용오차. Python float64 vs 보드 float32 차이라
 * cref/test_ae_int8.c와 같은 기준을 쓴다(혼합: atol + rtol*|기대값|). */
#define F32_ATOL  1e-4f
#define F32_RTOL  1e-4f

static uint32_t prv_cyc_to_us10(uint32_t cyc)   /* 사이클 -> 0.1us */
{
  return cyc * 10u / (SystemCoreClock / 1000000u);
}

/** med 사이클을 "12.3 us" 꼴로 찍는다. */
static void prv_print_row(const char *name, const dwt_stats_t *s)
{
  const uint32_t med10 = prv_cyc_to_us10(s->med);
  const uint32_t min10 = prv_cyc_to_us10(s->min);
  const uint32_t max10 = prv_cyc_to_us10(s->max);

  printf("     %-14s min=%6lu med=%6lu p99=%6lu max=%6lu cyc"
         "  (%lu.%lu / %lu.%lu / %lu.%lu us)\r\n",
         name,
         (unsigned long)s->min, (unsigned long)s->med,
         (unsigned long)s->p99, (unsigned long)s->max,
         (unsigned long)(min10 / 10u), (unsigned long)(min10 % 10u),
         (unsigned long)(med10 / 10u), (unsigned long)(med10 % 10u),
         (unsigned long)(max10 / 10u), (unsigned long)(max10 % 10u));
}

/* ------------------------------------------------------------------ 자가검증 */

/**
  * 보드 추론 == PC 기대값?  윈도우 하나에 대해 전 경로를 돌려 대조한다.
  * 정수 경로는 **비트정확**을 요구한다 - 정수 연산에 "거의 같음"은 없다.
  */
static bool prv_selftest_one(uint32_t s, bool *out_anom_i8, bool *out_anom_f32)
{
  int32_t err_i;
  float   err_f;
  bool    ok = true;

  ae_features(ae_tv_win[s], AE_WIN_LEN, s_feat);
  ae_normalize(s_feat, s_feat);

  ae_quantize_input(s_feat, s_q_in);
  ae_forward(s_q_in, s_q_out);
  err_i = ae_recon_error(s_q_in, s_q_out);

  ae_forward_f32(s_feat, s_recon);
  err_f = ae_recon_error_f32(s_feat, s_recon);

  *out_anom_i8  = (ae_is_anomaly(err_i) != 0);
  *out_anom_f32 = (ae_is_anomaly_f32(err_f) != 0);

  /* 정수 오차: 특징 추출이 float라 PC와 마지막 자리가 갈릴 여지가 원리상 있다.
   * cref 검증에서는 48개 전부 완전일치였으므로 여기서도 완전일치를 기대하되,
   * 어긋나면 값을 찍어서 "얼마나" 어긋났는지 보이게 한다. */
  if (err_i != ae_tv_err_int[s])
  {
    printf("     [%s] int8 오차 불일치: board=%ld pc=%ld\r\n",
           ae_tv_label[s], (long)err_i, (long)ae_tv_err_int[s]);
    ok = false;
  }
  if (fabsf(err_f - ae_tv_err_f32[s]) >
      (F32_ATOL + F32_RTOL * fabsf(ae_tv_err_f32[s])))
  {
    printf("     [%s] f32 오차 허용범위 밖: board=%d.%03d pc=%d.%03d (x1000)\r\n",
           ae_tv_label[s],
           (int)(err_f), (int)(fabsf(err_f - (float)(int)err_f) * 1000.0f),
           (int)(ae_tv_err_f32[s]),
           (int)(fabsf(ae_tv_err_f32[s] - (float)(int)ae_tv_err_f32[s]) * 1000.0f));
    ok = false;
  }

  /* 판정이 갈려야 할 두 샘플이다 - normal은 정상, 나머지는 이상으로 나와야 한다.
   * 오차값이 맞아도 임계값 상수가 틀리면 여기서 잡힌다. */
  {
    const bool want_anom = (ae_tv_err_int[s] > AE_THRESHOLD_INT);
    if (*out_anom_i8 != want_anom)
    {
      printf("     [%s] int8 판정 뒤집힘\r\n", ae_tv_label[s]);
      ok = false;
    }
  }

  return ok;
}

/* ------------------------------------------------------------------ 커널 비교 */

/**
  * 윈도우 하나로 네 단계를 각각 BENCH_N회 잰다.
  *
  * 한 반복 안에서 네 단계를 순서대로 도는 이유: 단계별로 루프를 따로 돌면
  * 각 루프가 자기 데이터만 캐시/파이프라인에 올려두고 돌아 실제 추론 경로보다
  * 유리한 조건이 된다. 실제로는 특징 -> 양자화 -> 순전파가 이어서 돈다.
  *
  * f32를 i8 **뒤에** 두는 건 의도적이다. 앞에 두면 f32가 s_feat를 갓 만들어진
  * 상태(캐시 hot)로 받고 i8은 그렇지 않아 순서가 곧 유리함이 된다. 뒤에 두면
  * f32가 불리한 쪽이므로, "f32가 이겼다"는 결과가 나오면 그건 순서 덕이 아니다.
  */
static void prv_bench_one(uint32_t s)
{
  dwt_stats_t st;
  uint32_t i;

  for (i = 0u; i < (BENCH_N + BENCH_WARMUP); i++)
  {
    uint32_t t0;
    const uint32_t k = (i >= BENCH_WARMUP) ? (i - BENCH_WARMUP) : 0u;

    t0 = dwt_now();
    ae_features(ae_tv_win[s], AE_WIN_LEN, s_feat);
    ae_normalize(s_feat, s_feat);
    const uint32_t c_feat = dwt_elapsed(t0);

    t0 = dwt_now();
    ae_quantize_input(s_feat, s_q_in);
    const uint32_t c_quant = dwt_elapsed(t0);

    t0 = dwt_now();
    ae_forward(s_q_in, s_q_out);
    (void)ae_recon_error(s_q_in, s_q_out);
    const uint32_t c_i8 = dwt_elapsed(t0);

    t0 = dwt_now();
    ae_forward_f32(s_feat, s_recon);
    (void)ae_recon_error_f32(s_feat, s_recon);
    const uint32_t c_f32 = dwt_elapsed(t0);

    if (i >= BENCH_WARMUP)
    {
      s_cyc_feat[k]  = c_feat;
      s_cyc_quant[k] = c_quant;
      s_cyc_i8[k]    = c_i8;
      s_cyc_f32[k]   = c_f32;
    }
  }

  printf("  [%s] n=%lu (워밍업 %lu 제외)\r\n",
         ae_tv_label[s], (unsigned long)BENCH_N, (unsigned long)BENCH_WARMUP);

  /* dwt_stats는 버퍼를 제자리 정렬한다(파괴적). 그래서 통계를 먼저 다 뽑고
   * CSV를 찍는 게 아니라, 하나씩 뽑아 바로 찍는다 - 원본이 더는 필요 없다. */
  dwt_stats(s_cyc_feat, BENCH_N, &st);
  prv_print_row("feat+norm", &st);
  dwt_csv_row_cfg("kernel_feat", ae_tv_label[s], &st);

  dwt_stats(s_cyc_quant, BENCH_N, &st);
  prv_print_row("quantize", &st);
  dwt_csv_row_cfg("kernel_quant", ae_tv_label[s], &st);

  dwt_stats(s_cyc_i8, BENCH_N, &st);
  const uint32_t med_i8 = st.med;
  prv_print_row("fwd int8", &st);
  dwt_csv_row_cfg("kernel_fwd_i8", ae_tv_label[s], &st);

  dwt_stats(s_cyc_f32, BENCH_N, &st);
  prv_print_row("fwd float32", &st);
  dwt_csv_row_cfg("kernel_fwd_f32", ae_tv_label[s], &st);

  /* 비율은 100배 정수로 찍는다 - float printf를 여기서 쓰면 그 자체가 무겁고,
   * newlib-nano는 %f를 링크에서 빼기도 한다. */
  {
    const uint32_t med_f32 = st.med;
    const unsigned long r = (med_f32 != 0u)
                          ? (unsigned long)((uint64_t)med_i8 * 100u / med_f32)
                          : 0u;
    printf("     -> int8/float32 중앙값 비 = %lu.%02lu  (%s)\r\n",
           r / 100u, r % 100u,
           (med_i8 < med_f32) ? "int8이 빠름" : "float32가 빠름");
  }
}

/* ------------------------------------------------------------------ 진입점 */

bool ae_bench(void)
{
  bool ok = true;
  uint32_t s;

  printf("\r\n--- AE 추론 자가검증 + 커널 비교 ---\r\n");
  printf("모델 %d->...->%d, 특징 %d, 윈도우 %d샘플(%d Hz), 임계값 int=%d\r\n",
         (int)ae_dim_in[0], (int)ae_dim_out[AE_N_LAYERS - 1],
         AE_N_FEATURES, AE_WIN_LEN, AE_FS_HZ, AE_THRESHOLD_INT);

  /* [1] 자가검증 */
  for (s = 0u; s < AE_TV_N; s++)
  {
    bool a_i8 = false, a_f32 = false;
    const bool one = prv_selftest_one(s, &a_i8, &a_f32);

    printf("[6] infer(%-8s): %s  판정 int8=%s float32=%s (기대 %s)\r\n",
           ae_tv_label[s], one ? "PASS" : "FAIL",
           a_i8 ? "이상" : "정상", a_f32 ? "이상" : "정상",
           (ae_tv_err_int[s] > AE_THRESHOLD_INT) ? "이상" : "정상");
    ok = ok && one;
  }

  /* [2] 커널 비교. 자가검증이 실패해도 돌린다 - 실패했을 때 사이클 분포가
   * 오히려 진단 정보가 된다(예: 한쪽이 비정상적으로 빠르면 루프가 안 돈 것). */
  printf("[7] kernel   : float32 vs int8 순전파 "
         "(-> measurements/kernel_compare.csv)\r\n");
  printf("tag,config,n,min,median,p99,max\r\n");
  for (s = 0u; s < AE_TV_N; s++)
  {
    prv_bench_one(s);
  }

  /* 입력 두 개의 결과가 다르면 추론 시간이 데이터에 의존한다는 뜻이고,
   * 그러면 WCET를 "한 입력으로 잰 max"로 말할 수 없다. 이 프로젝트에서
   * 그건 축 A의 논점이라 명시적으로 눈에 띄게 해둔다. */
  printf("     (입력별 차이가 크면 추론 시간이 데이터 의존적이라는 뜻 = WCET 논점)\r\n");

  printf("--- AE bench: %s ---\r\n", ok ? "PASS" : "FAIL");
  return ok;
}
