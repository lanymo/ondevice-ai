/* float32 오토인코더 순전파 — **대조군 전용**.
 *
 * 이 파일은 제품 경로가 아니다. [system-design.md §5](../../docs/system-design.md)의
 * 질문 하나에 답하기 위해서만 존재한다:
 *
 *     "이 모델 규모, 이 하드웨어(Cortex-M4F)에서 int8이 정말 이득인가?"
 *
 * 일반론은 "int8이 빠르다"지만 그 근거는 SIMD(CMSIS-NN `SMLAD`)를 전제한다.
 * 우리는 순수 C다. 반면 M4F에는 하드웨어 FPU가 있어서 float MAC은 `VMLA.F32`
 * 명령 하나고, int8 경로는 로드/부호확장/곱/누산에 **재양자화**(64비트 곱 +
 * 라운딩 시프트)까지 붙는다. 그래서 순수 C int8이 float보다 **느릴 수 있다**.
 * 어느 쪽이 나오든 그대로 기록한다 — measurements/kernel_compare.csv.
 *
 * int8 경로(ae_infer.h)와의 대응:
 *
 *     ae_forward       ↔  ae_forward_f32        (입력: q_in / z-score)
 *     ae_recon_error   ↔  ae_recon_error_f32    (**단위가 다르다**, 아래 주석)
 *     ae_is_anomaly    ↔  ae_is_anomaly_f32
 *
 * 특징 추출(ae_features)·정규화(ae_normalize)는 두 경로가 **공유**한다. 그래서
 * 계측할 때도 공통 비용은 분리해서 재야 한다 — 안 그러면 같은 비용이 두 숫자에
 * 다 섞여 정작 보려는 차이가 희석된다.
 *
 * 제약은 ae_infer.h와 같다: malloc 금지, 재진입 불가(파일 스코프 static 핑퐁 버퍼).
 */
#ifndef AE_INFER_F32_H
#define AE_INFER_F32_H

#include "model_weights.h"

#ifdef __cplusplus
extern "C" {
#endif

/* z-score → 재구성된 z-score. in과 out은 **겹치면 안 된다**.
 * 양자화가 없으므로 입력·출력이 같은 공간(z-score)에 그대로 있다. */
void ae_forward_f32(const float *in, float *out);

/* 재구성 오차 = mean((out[i] - in[i])^2).
 *
 * **정수 경로와 단위가 다르다.** 정수는 int8 격자 위의 제곱 *합*(AE_THRESHOLD_INT
 * = 4294), 이쪽은 z-score 공간의 제곱 *평균*(AE_THRESHOLD_F32 ≈ 0.932).
 * autoencoder.py recon_error()와 같은 정의라 Python 수치와 직접 비교된다.
 * 두 값을 같은 축에 그리거나 서로의 임계값에 넣지 말 것. */
float ae_recon_error_f32(const float *in, const float *out);

static inline int ae_is_anomaly_f32(float err) { return err > AE_THRESHOLD_F32; }

/* 윈도우 하나 → float 재구성 오차까지. int8의 ae_run_window와 짝. */
float ae_run_window_f32(const float *win, int win_len);

#ifdef __cplusplus
}
#endif

#endif /* AE_INFER_F32_H */
