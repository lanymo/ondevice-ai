#include "ae_infer.h"

#include <math.h>
#include <stddef.h>

/* 이 파일은 **측정 대상**이다(measurements/kernel_compare.csv, inference_cycles.csv).
 * CubeIDE의 Debug 구성은 -O0라, 그대로 재면 재는 게 커널이 아니라 -O0가 만들어내는
 * 스택 왕복이 된다 — 특히 int8 경로는 재양자화에 64비트 곱이 있어서 -O0에서
 * 레지스터 스필이 심하고, float 경로는 FPU 명령 하나라 왜곡의 방향도 다르다.
 * 즉 -O0 비교는 "int8 vs float"가 아니라 "누가 -O0에 더 취약한가"를 재게 된다.
 *
 * 그래서 빌드 구성과 무관하게 이 파일만 O2로 고정한다. 측정 코드와 배포 코드가
 * 같은 바이너리라는 점도 유지된다(다르면 잰 숫자가 제품의 숫자가 아니게 된다).
 * PC 검증 빌드(cref/Makefile)는 이미 -O2라 여기서 바뀌는 게 없다. */
#pragma GCC optimize ("O2")

/* ── 고정소수점 재양자화 ────────────────────────────────────────────────────
 *
 * gemmlowp / CMSIS-NN arm_nn_requantize 와 **비트정확 동일**해야 한다.
 * Python 쪽 짝은 model/quantize.py 의 requantize(). 둘이 갈리면
 * test_ae_int8 이 즉시 잡는다.
 *
 * 이식성 메모: 음수 우변 시프트(x >> n)는 C 표준상 구현정의지만 GCC/Clang은
 * 산술 시프트를 보장하고, Cortex-M4의 ASR 명령이 그것이다. Python 쪽도 산술
 * 시프트라 일치한다. 다른 컴파일러로 옮길 일이 생기면 여기부터 확인할 것.
 */

static int32_t sat_doubling_high_mul(int32_t a, int32_t b)
{
    if (a == INT32_MIN && b == INT32_MIN) {
        return INT32_MAX;                       /* 유일한 포화 케이스 */
    }
    const int64_t ab = (int64_t)a * (int64_t)b;
    /* round(ab / 2^31), 절반은 0에서 먼 쪽으로. C의 / 는 0 방향 절삭이라
     * nudge 부호를 ab 부호에 맞춰야 대칭이 된다. */
    const int64_t nudge = (ab >= 0) ? (int64_t)(1 << 30) : (int64_t)(1 - (1 << 30));
    const int64_t r = (ab + nudge) / ((int64_t)1 << 31);
    if (r > INT32_MAX) return INT32_MAX;
    if (r < INT32_MIN) return INT32_MIN;
    return (int32_t)r;
}

static int32_t rounding_divide_by_pot(int32_t x, int exponent)
{
    if (exponent == 0) {
        return x;
    }
    const int32_t mask = ((int32_t)1 << exponent) - 1;
    const int32_t remainder = x & mask;
    const int32_t threshold = (mask >> 1) + (x < 0 ? 1 : 0);
    return (x >> exponent) + ((remainder > threshold) ? 1 : 0);
}

int32_t ae_requantize(int32_t v, int32_t mult, int32_t shift)
{
    /* 현재 모델은 배율이 전부 1 미만이라 shift < 0 (우측 시프트)만 나온다.
     * 좌측 경로는 일반성을 위해 남겨두지만, 그 경우 v가 커지면 int32를 넘을 수
     * 있으니 새 모델을 export하면 test_ae_int8이 shift 부호를 다시 확인한다. */
    const int left = (shift > 0) ? shift : 0;
    const int right = (shift > 0) ? 0 : -shift;
    return rounding_divide_by_pot(sat_doubling_high_mul(v * ((int32_t)1 << left), mult),
                                  right);
}

/* ── 특징 추출 ──────────────────────────────────────────────────────────── */

void ae_features(const float *win, int win_len, float *feat_out)
{
    for (int c = 0; c < AE_N_CH; ++c) {
        const float *x = win + c;               /* 채널 c, stride = AE_N_CH */
        float sum = x[0];
        float mn = x[0];
        float mx = x[0];
        float mad = 0.0f;

        for (int n = 1; n < win_len; ++n) {
            const float v = x[(size_t)n * AE_N_CH];
            const float prev = x[(size_t)(n - 1) * AE_N_CH];
            const float d = v - prev;
            sum += v;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            mad += (d < 0.0f) ? -d : d;
        }

        const float mean = sum / (float)win_len;

        /* 2-pass: 평균을 뺀 뒤 제곱합. 큰 수끼리의 뺄셈을 피한다. */
        float ss = 0.0f;
        for (int n = 0; n < win_len; ++n) {
            const float d = x[(size_t)n * AE_N_CH] - mean;
            ss += d * d;
        }

        float *f = feat_out + (size_t)c * AE_N_FEAT_PER_CH;
        f[0] = mean;
        f[1] = sqrtf(ss / (float)win_len);
        f[2] = mx - mn;
        f[3] = mad / (float)(win_len - 1);
    }
}

void ae_normalize(const float *feat, float *out)
{
    for (int i = 0; i < AE_N_FEATURES; ++i) {
        out[i] = (feat[i] - ae_norm_mean[i]) / ae_norm_std[i];
    }
}

void ae_quantize_input(const float *norm, int8_t *q_out)
{
    for (int i = 0; i < AE_N_FEATURES; ++i) {
        int32_t q = (int32_t)roundf(norm[i] / AE_IN_SCALE) + AE_IN_ZP;
        if (q < -128) q = -128;
        if (q > 127) q = 127;
        q_out[i] = (int8_t)q;
    }
}

/* ── int8 순전파 ────────────────────────────────────────────────────────── */

void ae_forward(const int8_t *q_in, int8_t *q_out)
{
    /* 층간 핑퐁 버퍼. malloc 없음, 스택도 안 쓴다(태스크 스택 예산 보호).
     * 총 2 × AE_MAX_DIM 바이트 = 48 B. */
    static int8_t buf_a[AE_MAX_DIM];
    static int8_t buf_b[AE_MAX_DIM];

    const int8_t *src = q_in;
    int8_t *dst = buf_a;

    for (int l = 0; l < AE_N_LAYERS; ++l) {
        const int nin = (int)ae_dim_in[l];
        const int nout = (int)ae_dim_out[l];
        const int8_t *w = ae_w[l];
        const int32_t *b = ae_b[l];
        const int32_t in_zp = ae_in_zp[l];
        const int32_t out_zp = ae_out_zp[l];
        const int32_t mult = ae_mult[l];
        const int32_t shift = ae_shift[l];
        const int32_t lo = ae_act_min[l];
        const int32_t hi = ae_act_max[l];
        int8_t *out = (l == AE_N_LAYERS - 1) ? q_out : dst;

        for (int j = 0; j < nout; ++j) {
            const int8_t *wr = w + (size_t)j * (size_t)nin;
            int32_t acc = b[j];                 /* bias는 int32 (누산기와 같은 스케일) */
            for (int i = 0; i < nin; ++i) {
                /* int8 × int8 → int32 누산. 24항 × 255 × 127 ≈ 7.8e5 이라 여유. */
                acc += ((int32_t)src[i] - in_zp) * (int32_t)wr[i];
            }
            acc = ae_requantize(acc, mult, shift) + out_zp;
            if (acc < lo) acc = lo;             /* ReLU는 clamp 하한으로 흡수 */
            if (acc > hi) acc = hi;
            out[j] = (int8_t)acc;
        }

        if (l != AE_N_LAYERS - 1) {
            src = dst;
            dst = (dst == buf_a) ? buf_b : buf_a;
        }
    }
}

int32_t ae_recon_error(const int8_t *q_in, const int8_t *q_out)
{
    int32_t err = 0;
    for (int i = 0; i < AE_N_FEATURES; ++i) {
        const int32_t d = (int32_t)q_out[i] - (int32_t)q_in[i];
        err += d * d;
    }
    return err;
}

int32_t ae_run_window(const float *win, int win_len)
{
    static float feat[AE_N_FEATURES];
    static int8_t q_in[AE_N_FEATURES];
    static int8_t q_out[AE_N_FEATURES];

    ae_features(win, win_len, feat);
    ae_normalize(feat, feat);
    ae_quantize_input(feat, q_in);
    ae_forward(q_in, q_out);
    return ae_recon_error(q_in, q_out);
}