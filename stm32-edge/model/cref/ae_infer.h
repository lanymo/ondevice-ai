/* int8 오토인코더 이상탐지 — 순수 C99 추론.
 *
 * **이 파일과 ae_infer.c 는 그대로 펌웨어에 들어간다.** W3에서 CubeIDE 프로젝트에
 * model_weights.h 와 함께 복사하면 된다. 그래서 지켜야 할 제약이 있다:
 *
 *   - malloc 금지 (CLAUDE.md). 모든 버퍼는 파일 스코프 static 또는 호출자 제공.
 *   - libm은 sqrtf 하나만 쓴다 (특징 계산의 표준편차). 추론 경로엔 float 나눗셈도 없다.
 *   - HAL/FreeRTOS/CMSIS 의존 없음 → PC에서 그대로 컴파일해 검증할 수 있다.
 *     실제로 test_ae_int8.c가 Python 정수 시뮬과 비트 단위로 대조한다.
 *
 * **재진입 불가**: ae_forward()가 파일 스코프 static 핑퐁 버퍼를 쓴다.
 * 추론은 추론 태스크 하나에서만 호출할 것 (축 A의 태스크 설계와 일관).
 *
 * 파이프라인:
 *     링버퍼 윈도우 → ae_features → ae_normalize → ae_quantize_input
 *                  → ae_forward → ae_recon_error → ae_is_anomaly
 */
#ifndef AE_INFER_H
#define AE_INFER_H

#include <stdint.h>

#include "model_weights.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 윈도우 → 24차원 특징 (물리 단위 그대로).
 *
 * win: [win_len][AE_N_CH] 채널 인터리브 연속 배열. win[n*AE_N_CH + c].
 *      링버퍼가 감겨 있으면 호출 전에 펴서 넘길 것.
 * win_len: 보통 AE_WIN_LEN. 2 이상이어야 한다(mad가 차분을 쓴다).
 *
 * 표준편차는 **2-pass**로 계산한다 — az처럼 평균 1g에 표준편차 0.005g인 채널에서
 * 1-pass 공식 sqrt(E[x²]-E[x]²)은 float32 유효자릿수가 날아간다. features.py 참고.
 */
void ae_features(const float *win, int win_len, float *feat_out);

/* 특징 → z-score. feat와 out은 같은 포인터여도 된다. */
void ae_normalize(const float *feat, float *out);

/* z-score → int8. 반올림은 roundf(0에서 먼 쪽) — Python 쪽도 여기 맞춰져 있다. */
void ae_quantize_input(const float *norm, int8_t *q_out);

/* int8 순전파. q_in과 q_out은 **겹치면 안 된다**. */
void ae_forward(const int8_t *q_in, int8_t *q_out);

/* 재구성 오차 = sum((q_out[i] - q_in[i])^2). 순수 int32, 스케일 환산 없음.
 * 최대값은 AE_ERR_MAX(=1560600)라 int32 오버플로 여지 없음. */
int32_t ae_recon_error(const int8_t *q_in, const int8_t *q_out);

/* 임계값 비교. 1이면 이상. */
static inline int ae_is_anomaly(int32_t err) { return err > AE_THRESHOLD_INT; }

/* 윈도우 하나 → 재구성 오차까지 한 번에. 중간 결과가 필요 없을 때.
 * q_in_out/q_out_out에 NULL을 주면 내부 static 버퍼를 쓴다(디버깅용으로 꺼내볼 때만 비NULL). */
int32_t ae_run_window(const float *win, int win_len);

/* 검증·계측용으로 노출. 펌웨어 로직에선 쓸 일 없다. */
int32_t ae_requantize(int32_t v, int32_t mult, int32_t shift);

#ifdef __cplusplus
}
#endif

#endif /* AE_INFER_H */