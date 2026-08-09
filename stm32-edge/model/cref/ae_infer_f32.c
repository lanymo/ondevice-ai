#include "ae_infer_f32.h"

#include "ae_infer.h"   /* ae_features / ae_normalize — 두 경로 공용 */

#include <stddef.h>

/* ae_infer.c와 같은 이유로 O2 고정 — 두 커널을 비교하려면 **같은 조건**이어야 한다.
 * 한쪽만 -O0면 그 비교는 최적화 수준을 재는 것이지 산술을 재는 게 아니다. */
#pragma GCC optimize ("O2")

/* 층간 핑퐁 버퍼. int8 쪽(ae_infer.c)과 같은 이유로 스택이 아니라 static이다 —
 * 태스크 스택 예산을 여기서 먹으면 STK_INFER를 키워야 한다.
 * 2 × 24 × 4 B = 192 B (int8 쪽 48 B의 4배. 이것도 비교 대상 숫자다). */
static float s_buf_a[AE_MAX_DIM];
static float s_buf_b[AE_MAX_DIM];

void ae_forward_f32(const float *in, float *out)
{
    const float *src = in;
    float *dst = s_buf_a;

    for (int l = 0; l < AE_N_LAYERS; ++l) {
        const int nin = (int)ae_dim_in[l];
        const int nout = (int)ae_dim_out[l];
        const float *w = ae_wf[l];
        const float *b = ae_bf[l];
        const int relu = (int)ae_is_relu[l];
        float *o = (l == AE_N_LAYERS - 1) ? out : dst;

        for (int j = 0; j < nout; ++j) {
            /* 가중치 레이아웃은 int8 쪽과 동일한 [out][in] — 행 하나가 출력 뉴런
             * 하나다. 두 커널이 같은 메모리 접근 패턴을 갖게 해야 비교가 공정하다.
             * (한쪽만 캐시 친화적이면 재는 게 산술이 아니라 접근 패턴이 된다.) */
            const float *wr = w + (size_t)j * (size_t)nin;
            float acc = b[j];
            for (int i = 0; i < nin; ++i) {
                acc += wr[i] * src[i];
            }
            /* int8 쪽은 ReLU가 clamp 하한(act_min)에 흡수돼 있지만 float는
             * clamp 자체가 없으므로 여기서 명시적으로 적용한다. */
            o[j] = (relu && (acc < 0.0f)) ? 0.0f : acc;
        }

        if (l != AE_N_LAYERS - 1) {
            src = dst;
            dst = (dst == s_buf_a) ? s_buf_b : s_buf_a;
        }
    }
}

float ae_recon_error_f32(const float *in, const float *out)
{
    float err = 0.0f;
    for (int i = 0; i < AE_N_FEATURES; ++i) {
        const float d = out[i] - in[i];
        err += d * d;
    }
    /* 평균 — autoencoder.py recon_error()가 mean(axis=1)이라 그 정의를 따른다.
     * 정수 경로가 합인 건 나눗셈을 피하려는 것이고, 여기선 FPU가 있으니 무료다. */
    return err / (float)AE_N_FEATURES;
}

float ae_run_window_f32(const float *win, int win_len)
{
    static float feat[AE_N_FEATURES];
    static float recon[AE_N_FEATURES];

    ae_features(win, win_len, feat);
    ae_normalize(feat, feat);
    ae_forward_f32(feat, recon);
    return ae_recon_error_f32(feat, recon);
}
