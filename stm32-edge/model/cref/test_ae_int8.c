/* Python 파이프라인 ↔ C 추론 대조 검증 (PC에서 실행).
 *
 * 보드에 올리기 전에 커널 버그를 여기서 잡는 게 목적이다. 세 종류의 검사를 나눠서 한다:
 *
 *   [정수 경로] Python이 만든 q_in을 그대로 먹여 q_out과 재구성오차를 **비트 단위로**
 *              일치시킨다. 여기서 1이라도 틀리면 재양자화/누산/clamp 중 하나가 틀린 것.
 *
 *   [float 경로] 특징·정규화는 Python이 float64, C가 float32라 비트 일치가 애초에
 *              불가능하다. 상대오차 허용치로 본다.
 *
 *   [end-to-end] C가 자기 float 경로로 만든 q_in이 Python의 q_in과 몇 개나 다른지 센다.
 *              반올림 경계에서 ±1은 원리상 나올 수 있으므로 개수만 보고하고,
 *              **그게 최종 이상판정을 뒤집는지**를 따로 확인한다. 뒤집히면 FAIL.
 *
 *   [f32 대조군] system-design.md §5의 float 커널(ae_infer_f32.c). 제품 경로는
 *              아니지만 **속도를 비교하려면 먼저 맞아야 한다** — 검증 안 된 커널이
 *              빠른 건 아무 의미가 없다. Python float64 순전파와 허용오차로 본다.
 *
 * 사용: ./test_ae_int8 [testvec.bin]
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ae_infer.h"
#include "ae_infer_f32.h"

#define MAGIC   0x56544541
#define VERSION 2

/* 특징은 **상대**오차로 본다 (물리 단위라 크기가 채널마다 다르다). */
#define FEAT_RTOL  2e-4f

/* z-score 허용오차는 **양자화 LSB(AE_IN_SCALE)에 비례**시킨다.
 *
 * 처음엔 절대값 1e-3으로 박아뒀는데, 실측 데이터로 재학습하자 실패했다(2026-08-08).
 * 원인은 정밀도 저하가 아니라 **척도 변화**다: 실제 정상 데이터가 합성보다 훨씬
 * 일관돼서 norm_std가 10배 작아졌고(ax.std 특징의 std가 1.2e-4), 같은 물리 오차가
 * z에서 그만큼 증폭된다. 절대 기준을 쓰면 모델을 재학습할 때마다 이 상수를 손으로
 * 다시 맞춰야 하고, 그러다 보면 "통과할 때까지 느슨하게 하는" 짓을 하게 된다.
 *
 * 실제로 중요한 건 절대 z가 아니라 **그 오차가 q_in을 흔드느냐**다. 1 LSB의 절반을
 * 넘으면 양자화 결과가 바뀐다. 그래서 0.1 LSB를 기준으로 잡는다 — 판정이 바뀌는
 * 선(0.5)보다 5배 엄격하면서, 모델이 바뀌어도 자동으로 따라간다.
 * (실측 마진: 최대 오차가 0.0137 LSB, 즉 기준의 1/7.) */
#define NORM_ATOL  (0.1f * AE_IN_SCALE)

/* 아래는 원래 절대 기준을 쓰던 이유의 기록이다 — 상대오차가 왜 안 되는지는
 * 지금도 유효하므로 남긴다.
 *   az.mean은 평균 0.99968 g에 표준편차 0.00056 g다. z = (x - 0.99968)/0.00056 에서
 *   분자가 큰 수끼리의 뺄셈이라 float32 유효자릿수가 날아간다(features.py가 std를
 *   2-pass로 짠 것과 같은 현상). z가 0 근처인 정상 샘플에선 절대오차가 아무리 작아도
 *   상대오차가 발산하므로, 상대오차 기준 자체가 의미가 없다.
 *   z는 이미 표준화된 양(스케일 1)이라 절대오차가 옳은 기준이다.
 *
 * 정밀도 바닥의 이론값: float32 eps(1.2e-7) / (std/mean 비율) = 1.2e-7/5.6e-4 ≈ 2e-4.
 * 이게 양자화 LSB의 몇 분의 1인지를 아래에서 같이 찍는다 — 0.5 LSB를 넘으면 q_in이
 * 흔들리기 시작하므로 그때 대책(오프셋 선차감 등)을 세우면 된다. */

/* f32 대조군: Python float64 vs C float32라 비트 일치는 불가능하다.
 *
 * 기준을 **표본 안 최대 크기에 상대적으로** 잡는다. 개별 값에 대한 상대오차는
 * ReLU 뒤 0에 붙은 출력에서 발산하고, 고정 절대값은 위 z-score와 같은 이유로
 * 모델이 바뀌면 무너진다(실데이터에서 z가 최대 3만까지 간다).
 *
 * 1e-5를 고른 근거: float32 eps는 1.19e-7이고 24항 4층 누산이니 바닥은 1e-6 언저리다.
 * 거기에 10배 여유. 실측 오차는 recon 2.27e-07, MSE 1.52e-07 — **정확히 eps 수준**이라
 * 커널이 float32가 낼 수 있는 최대 정확도로 맞고 있다는 뜻이고, 기준 대비 40배 여유다.
 * 이보다 크게 틀리면 그건 반올림이 아니라 로직 버그다. */
#define RECON_F32_RTOL  1e-5f

static void *xmalloc(size_t n)
{
    void *p = malloc(n);
    if (!p) { fprintf(stderr, "메모리 부족\n"); exit(2); }
    return p;
}

static void rd(FILE *f, void *p, size_t n, const char *what)
{
    if (fread(p, 1, n, f) != n) {
        fprintf(stderr, "testvec 읽기 실패: %s\n", what);
        exit(2);
    }
}

static float relerr(float got, float want)
{
    const float d = fabsf(got - want);
    const float scale = (fabsf(want) > 1e-6f) ? fabsf(want) : 1e-6f;
    return d / scale;
}

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "testvec.bin";
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "열 수 없음: %s\n", path); return 2; }

    int32_t hdr[6];
    rd(f, hdr, sizeof hdr, "header");
    if (hdr[0] != MAGIC || hdr[1] != VERSION) {
        fprintf(stderr, "magic/version 불일치 — export_header.py를 다시 돌려라\n");
        return 2;
    }
    if (hdr[2] <= 0 || hdr[3] != AE_WIN_LEN || hdr[4] != AE_N_CH || hdr[5] != AE_N_FEATURES) {
        fprintf(stderr, "model_weights.h와 testvec.bin 설정이 다르다 "
                        "(win %d/%d, ch %d/%d, feat %d/%d)\n",
                (int)hdr[3], AE_WIN_LEN, (int)hdr[4], AE_N_CH,
                (int)hdr[5], AE_N_FEATURES);
        return 2;
    }

    const size_t N = (size_t)hdr[2];              /* 샘플 수 */
    const size_t NF = (size_t)AE_N_FEATURES;      /* 샘플당 특징 수 */
    const size_t NW = (size_t)AE_WIN_LEN * (size_t)AE_N_CH;  /* 샘플당 윈도우 원소 수 */

    float *windows = xmalloc(N * NW * sizeof(float));
    float *feats = xmalloc(N * NF * sizeof(float));
    float *norms = xmalloc(N * NF * sizeof(float));
    int8_t *q_in = xmalloc(N * NF);
    int8_t *q_out = xmalloc(N * NF);
    int32_t *err = xmalloc(N * sizeof(int32_t));
    float *reconf = xmalloc(N * NF * sizeof(float));
    float *errf = xmalloc(N * sizeof(float));
    rd(f, windows, N * NW * sizeof(float), "windows");
    rd(f, feats, N * NF * sizeof(float), "feats");
    rd(f, norms, N * NF * sizeof(float), "norms");
    rd(f, q_in, N * NF, "q_in");
    rd(f, q_out, N * NF, "q_out");
    rd(f, err, N * sizeof(int32_t), "err");
    rd(f, reconf, N * NF * sizeof(float), "recon_f32");
    rd(f, errf, N * sizeof(float), "err_f32");
    fclose(f);

    printf("testvec: %s — 샘플 %zu개, 윈도우 %d×%d, 특징 %d\n",
           path, N, AE_WIN_LEN, AE_N_CH, AE_N_FEATURES);
    printf("모델: %d→...→%d, 임계값 %d, 가중치+상수 ROM %zu B\n\n",
           (int)ae_dim_in[0], (int)ae_dim_out[AE_N_LAYERS - 1], AE_THRESHOLD_INT,
           sizeof ae_w0 + sizeof ae_w1 + sizeof ae_w2 + sizeof ae_w3 +
           sizeof ae_b0 + sizeof ae_b1 + sizeof ae_b2 + sizeof ae_b3 +
           sizeof ae_norm_mean + sizeof ae_norm_std);

    int fail = 0;

    /* shift 부호 전제 확인 — 좌측 시프트가 생기면 ae_requantize의 int32 좌측
     * 경로에 오버플로 위험이 열린다(현재 모델은 전부 음수라 우측 시프트만 탄다). */
    for (int l = 0; l < AE_N_LAYERS; ++l) {
        if (ae_shift[l] > 0) {
            printf("[경고] layer %d의 shift가 양수(%d)다. ae_requantize 좌측 경로의 "
                   "오버플로를 재검토할 것.\n", l, (int)ae_shift[l]);
        }
    }

    /* ── 1. 정수 경로: 비트정확이어야 한다 ────────────────────────────── */
    size_t bad_out = 0, bad_err = 0;
    int8_t c_out[AE_N_FEATURES];
    for (size_t s = 0; s < N; ++s) {
        ae_forward(q_in + s * NF, c_out);
        if (memcmp(c_out, q_out + s * NF, NF) != 0) {
            if (bad_out == 0) {
                printf("[FAIL] 샘플 %zu q_out 불일치:\n  C  :", s);
                for (size_t i = 0; i < NF; ++i) printf(" %4d", c_out[i]);
                printf("\n  py :");
                for (size_t i = 0; i < NF; ++i) printf(" %4d", q_out[s * NF + i]);
                printf("\n");
            }
            ++bad_out;
        }
        const int32_t e = ae_recon_error(q_in + s * NF, c_out);
        if (e != err[s]) {
            if (bad_err == 0)
                printf("[FAIL] 샘플 %zu 재구성오차: C=%d py=%d\n",
                       s, (int)e, (int)err[s]);
            ++bad_err;
        }
    }
    printf("[정수 경로]   q_out 불일치 %zu/%zu, 오차 불일치 %zu/%zu  →  %s\n",
           bad_out, N, bad_err, N,
           (bad_out || bad_err) ? "FAIL" : "OK (비트정확)");
    fail += (bad_out != 0) + (bad_err != 0);

    /* ── 2. float 경로: 허용오차 ──────────────────────────────────────── */
    float c_feat[AE_N_FEATURES], c_norm[AE_N_FEATURES];
    float max_fe = 0.0f, max_ne = 0.0f;
    size_t bad_feat = 0, bad_norm = 0;
    for (size_t s = 0; s < N; ++s) {
        ae_features(windows + s * NW, AE_WIN_LEN, c_feat);
        ae_normalize(c_feat, c_norm);
        for (size_t i = 0; i < NF; ++i) {
            const float fe = relerr(c_feat[i], feats[s * NF + i]);
            const float ne = fabsf(c_norm[i] - norms[s * NF + i]);
            if (fe > max_fe) max_fe = fe;
            if (ne > max_ne) max_ne = ne;
            if (fe > FEAT_RTOL) ++bad_feat;
            if (ne > NORM_ATOL) ++bad_norm;
        }
    }
    printf("[float 경로]  특징 최대 상대오차 %.3e (허용 %.0e, 초과 %zu)\n"
           "              z-score 최대 절대오차 %.3e (허용 %.0e, 초과 %zu) "
           "= 양자화 LSB의 %.4f배\n"
           "              →  %s\n",
           (double)max_fe, (double)FEAT_RTOL, bad_feat,
           (double)max_ne, (double)NORM_ATOL, bad_norm,
           (double)(max_ne / AE_IN_SCALE),
           (bad_feat || bad_norm) ? "FAIL" : "OK");
    fail += (bad_feat != 0) + (bad_norm != 0);

    /* ── 3. end-to-end: C 자체 경로 vs Python ─────────────────────────── */
    size_t qin_diff = 0, decision_diff = 0, err_exact = 0;
    double max_err_rel = 0.0;
    for (size_t s = 0; s < N; ++s) {
        int8_t c_q[AE_N_FEATURES];
        ae_features(windows + s * NW, AE_WIN_LEN, c_feat);
        ae_normalize(c_feat, c_norm);
        ae_quantize_input(c_norm, c_q);
        for (size_t i = 0; i < NF; ++i)
            if (c_q[i] != q_in[s * NF + i]) ++qin_diff;

        const int32_t e = ae_run_window(windows + s * NW, AE_WIN_LEN);
        if (e == err[s]) ++err_exact;
        const double rel = fabs((double)e - (double)err[s]) /
                           ((err[s] != 0) ? (double)err[s] : 1.0);
        if (rel > max_err_rel) max_err_rel = rel;
        if (ae_is_anomaly(e) != (err[s] > AE_THRESHOLD_INT)) ++decision_diff;
    }
    printf("[end-to-end]  q_in 차이 %zu/%zu개 값, 오차 완전일치 %zu/%zu, "
           "최대 상대차 %.3e, **이상판정 뒤집힘 %zu/%zu**  →  %s\n",
           qin_diff, N * NF, err_exact, N, max_err_rel, decision_diff, N,
           decision_diff ? "FAIL" : "OK");
    fail += (decision_diff != 0);

    /* ── 4. f32 대조군: Python float64 순전파와 대조 ──────────────────── */
    {
        float c_recon[AE_N_FEATURES];
        float max_re = 0.0f, max_ee = 0.0f;
        size_t bad_recon = 0, bad_ef = 0, flip = 0;

        float max_rel = 0.0f;

        for (size_t s = 0; s < N; ++s) {
            /* Python이 준 z-score를 그대로 먹인다 — 특징 추출 오차가 섞이면
             * 커널 자체가 맞는지를 못 본다(그건 위 [float 경로]에서 이미 봤다). */
            ae_forward_f32(norms + s * NF, c_recon);

            /* 이 표본의 크기 척도. 개별 값이 아니라 벡터 전체의 최대 크기를 쓰는 건,
             * 누산 오차가 "그 뉴런의 출력 크기"가 아니라 "층을 지나온 값들의 크기"에서
             * 나오기 때문이다. ReLU로 0이 된 출력에도 같은 크기의 오차가 실린다. */
            float mag = 1.0f;
            for (size_t i = 0; i < NF; ++i) {
                const float a = fabsf(reconf[s * NF + i]);
                if (a > mag) mag = a;
            }

            for (size_t i = 0; i < NF; ++i) {
                const float want = reconf[s * NF + i];
                const float d = fabsf(c_recon[i] - want);
                if (d > max_re) max_re = d;
                if (d / mag > max_rel) max_rel = d / mag;
                if (d > RECON_F32_RTOL * mag) ++bad_recon;
            }
            const float e = ae_recon_error_f32(norms + s * NF, c_recon);
            const float de = fabsf(e - errf[s]);
            const float emag = (fabsf(errf[s]) > 1.0f) ? fabsf(errf[s]) : 1.0f;
            if (de > max_ee) max_ee = de;
            if (de / emag > max_rel) max_rel = de / emag;
            if (de > RECON_F32_RTOL * emag) ++bad_ef;
            if (ae_is_anomaly_f32(e) != (errf[s] > AE_THRESHOLD_F32)) ++flip;
        }
        printf("[f32 대조군] 최대 **상대**오차 %.3e (허용 %.0e = float32 eps의 %.0f배)\n",
               (double)max_rel, (double)RECON_F32_RTOL,
               (double)(RECON_F32_RTOL / 1.19e-7f));
        printf("             재구성 최대 절대오차 %.3e (초과 %zu/%zu), "
               "MSE 최대 절대오차 %.3e (초과 %zu/%zu)\n"
               "              이상판정 뒤집힘 %zu/%zu (임계값 %.4f, MSE 단위)  →  %s\n",
               (double)max_re, bad_recon, N * NF,
               (double)max_ee, bad_ef, N, flip, N,
               (double)AE_THRESHOLD_F32,
               (bad_recon || bad_ef || flip) ? "FAIL" : "OK");
        fail += (bad_recon != 0) + (bad_ef != 0) + (flip != 0);
    }

    /* ── 요약 ─────────────────────────────────────────────────────────── */
    size_t n_anom = 0, n_anom_f = 0, agree = 0;
    for (size_t s = 0; s < N; ++s) {
        const int ai = (err[s] > AE_THRESHOLD_INT);
        const int af = (errf[s] > AE_THRESHOLD_F32);
        n_anom += (size_t)ai;
        n_anom_f += (size_t)af;
        agree += (size_t)(ai == af);
    }
    printf("\n테스트벡터 구성: 이상 판정 %zu / 정상 판정 %zu (임계값 %d)\n",
           n_anom, N - n_anom, AE_THRESHOLD_INT);
    /* 두 경로가 몇 개나 같은 판정을 내리는지 — 양자화가 실제로 무엇을 바꿨는지의
     * 표본 크기 48짜리 스냅숏이다. 전수 수치는 measurements/accuracy.csv를 볼 것. */
    printf("int8 vs float32 판정 일치: %zu/%zu (float 쪽 이상 %zu개)\n",
           agree, N, n_anom_f);

    free(windows); free(feats); free(norms); free(q_in); free(q_out); free(err);
    free(reconf); free(errf);
    printf("\n%s\n", fail ? "=== 실패 ===" : "=== 전부 통과 ===");
    return fail ? 1 : 0;
}
