# 고정소수점 · int8 양자화 기초 — 축 B(삼성)의 온디바이스 추론

> 학습 자료(축 B). 왜 MCU에서 float가 아니라 int8로 추론하는가, float 모델을 어떻게 int8로
> 바꾸는가. [measurement-methodology.md](measurement-methodology.md)와 함께 온디바이스 추론의 두 축.

---

## 1. 왜 int8인가 — MCU에서의 이유

F411은 FPU가 있어 float도 돌지만, 그래도 int8을 쓰는 이유:

- **메모리**: 가중치가 float32→int8이면 **4배 작아진다.** 512KB 플래시·128KB SRAM에선 결정적.
- **속도**: int8 곱셈-누산은 **SIMD로 한 명령에 여러 개**(CMSIS-NN의 SMLAD류 DSP 명령)를 처리.
- **에너지**: 적은 비트·적은 메모리 이동 = 적은 에너지(엣지 AI의 핵심 제약).
- 실제 엣지/NPU(삼성 포함)가 int8(또는 그 이하)로 도는 이유가 이것. → 축 B 서사와 직결.

---

## 2. 핵심 아이디어 — 실수를 정수로 "스케일"해서 표현

양자화는 **아핀(affine) 매핑** 하나다:

```
real ≈ scale × (q − zero_point)
```

- `q`: 실제로 저장하는 int8 값(−128~127).
- `scale`(float): 한 스텝이 실수로 얼마인가(= (max−min)/255 류).
- `zero_point`(int): 실수 0에 대응하는 정수. **대칭(symmetric)** 양자화면 zero_point=0으로 고정.

> CS 연결: 이건 **고정소수점(fixed-point)** 표현이다. 컴퓨터구조에서 배운 정수/부동소수점의
> 중간 — "정수인데 스케일을 곱해 실수처럼 쓴다". zero_point는 부호 없는 표현의 바이어스와 비슷.

- **per-tensor vs per-channel**: 텐서 전체에 scale 하나 vs 출력 채널마다 scale 하나(정확도↑, CMSIS-NN 지원).
- **대칭 vs 비대칭**: 가중치는 보통 대칭(zero_point=0), 활성값은 비대칭이 흔함.

---

## 3. int8 dense/conv 레이어가 도는 방식

핵심은 **곱은 int8, 누산은 int32**:

1. 입력 q_in(int8) × 가중치 q_w(int8) → 곱은 최대 16비트, 이를 **int32 누산기**에 더한다.
   (수백~수천 개를 더하므로 int8 누산기면 오버플로 → int32 필수. **컴퓨터구조의 오버플로 그 자체.**)
2. bias(int32) 더함.
3. **재양자화(requantize)**: int32 누산값에 (scale_in·scale_w/scale_out)을 곱해 다시 int8로 내림.
   이 곱셈은 보통 "정수 곱 + 비트시프트"(fixed-point multiplier + shift)로 구현 → 나눗셈/float 회피.
4. 활성함수(ReLU 등)는 클램핑으로 처리.

CMSIS-NN의 `arm_fully_connected_s8`, `arm_convolve_s8`가 정확히 이 과정을 SIMD로 한다.

---

## 4. float 모델 → int8 만들기

- **PTQ(Post-Training Quantization)**: 학습된 float 모델을 사후 양자화. 이 프로젝트의 기본(간단).
  - **캘리브레이션**: 대표 입력 몇 배치를 흘려 각 텐서의 값 범위(min/max 또는 퍼센타일)를 수집 →
    scale/zero_point 산정. 아웃라이어에 약하면 퍼센타일(예: 99.9%) 클리핑.
- **QAT(Quantization-Aware Training)**: 학습 중 양자화를 시뮬레이션(fake-quant)해 정확도 회복.
  PTQ로 정확도가 부족할 때만. MVP엔 과함.
- 파이프라인: `train_mlp.py`(float 학습) → `export_int8.py`(PTQ + `model_weights.h` 생성) →
  PC에서 순수 C 순전파가 레퍼런스와 일치하는지 검증([mini-infer](../../mini-infer) 하네스 재사용).

---

## 5. 정확도 손실 — 어디서 오고 얼마나 감수하나

- 손실원: 값 범위 밖 클리핑, 스텝 반올림, 활성값 분포의 긴 꼬리.
- 목표: **float 대비 정확도 −Xp 이내**(PLAN.md DoD). 부족하면 클래스 수 축소(정상/이상 2클래스)로 여유 확보.
- 측정: `measurements/accuracy.csv`에 float vs int8 정확도를 나란히. **"양자화했더니 정확도 이만큼만
  떨어졌다"가 축 B의 수치.**

---

## 6. 초심자 함정

1. 누산기를 int8/int16로 → 오버플로로 결과가 쓰레기. **누산은 int32.**
2. 캘리브레이션 데이터가 실제 분포와 다름 → scale이 엉뚱해 정확도 급락.
3. 학습 축 방향 ≠ 추론 축 방향(IMU 장착 방향) → 모델이 통째로 무의미(quantization 이전 문제).
4. per-tensor로 충분한데 per-channel로 과설계, 또는 그 반대.
5. requantize를 float 나눗셈으로 → MCU에서 느림. 정수 곱+시프트로.

---

## 7. 자기 점검

1. `real ≈ scale × (q − zero_point)`에서 scale과 zero_point는 각각 무엇을 정하나?
2. int8 곱셈인데 누산을 왜 int32로 하나? (컴퓨터구조 용어로)
3. PTQ와 QAT의 차이, 그리고 이 프로젝트가 PTQ를 고른 이유는?
4. 캘리브레이션이 하는 일은?
5. 양자화 정확도 손실을 어떻게 수치로 보고하나?

---

### 더 읽을 것
- Jacob et al. (2018), *Quantization and Training of Neural Networks for Integer-Arithmetic-Only Inference* — affine 양자화 원전
- ARM CMSIS-NN 문서 — `*_s8` 커널의 입출력 양자화 규약(정확한 scale/shift 표현)
- gemmlowp의 "quantization" 문서 — 정수 곱+시프트 requantize 구현 관점