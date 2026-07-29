# 축 B 모델 파이프라인 — IMU 예지보전 int8 오토인코더

> **주차**: W2 (B트랙, board-independent). PLAN.md 110행의
> "Python 소형 모델 학습 + int8 export → `model_weights.h`, 순수 C 순전파를 PC에서 먼저 검증".
>
> **선행 개념노트**: [quantization-basics.md](../docs/quantization-basics.md) ·
> [anomaly-detection.md](../docs/anomaly-detection.md) ·
> [timeseries-windowing.md](../docs/timeseries-windowing.md)

센서 진동 → 통계 특징 → **int8 오토인코더 재구성 오차** → 임계값 → 이상 판정.
학습·양자화는 Python, 추론은 **펌웨어에 그대로 들어가는 순수 C**. 둘이 갈라지지 않는다는 걸
PC에서 비트 단위로 검증한 뒤 보드에 올린다.

---

## ⚠️ 지금 데이터는 합성이다

W2 시점에 MPU6050 브링업이 끝나지 않아 **`synth_data.py`가 만든 proxy 데이터**로 돌아간다.
PLAN.md §5 리스크표의 "추론 입력을 합성 버퍼로 대체" 경로다. 아래 수치는 전부 그 전제 위에 있다.

**실측으로 갈아끼우는 법**: 보드 UART로 세션마다 CSV 하나씩 뽑아 `data/`에 넣고
`data/manifest.csv`에 `파일,split,label` 한 줄씩 추가한다. 그 위 코드는 한 줄도 안 고친다.

```csv
# t_ms,ax,ay,az,gx,gy,gz   (가속도 g, 자이로 dps)
0.0,0.0203,-0.0114,0.9981,0.412,-0.233,0.051
```

세션을 **여러 개 짧게** 뽑을 것 — 이유는 아래 "설계에서 실제로 문제였던 것" §1.

---

## 빠른 실행

```bash
./run_all.sh          # 데이터 생성 → 학습 → 평가 → export → C검증 → 그래프
```

의존성은 **numpy와 matplotlib뿐**(그래프만 matplotlib). torch/sklearn 안 쓴다 —
파라미터 724개짜리 모델에 과하고, 리포 규칙이 외부 의존성 최소화다. 역전파는 직접 짰다.

개별 실행:

| 명령 | 하는 일 | 산출물 |
|---|---|---|
| `python3 synth_data.py` | 합성 IMU 세션 27개 | `data/*.csv`, `data/manifest.csv` |
| `python3 train.py` | float32 AE 학습 + 임계값 | `artifacts/ae_float.npz` |
| `python3 evaluate.py [--sweep]` | float vs int8 정확도 | `../measurements/accuracy.csv` |
| `python3 export_header.py` | 헤더 + 테스트벡터 | `cref/model_weights.h`, `cref/testvec.bin` |
| `make -C cref test` | **C 순전파 비트정확 검증** | 통과/실패 |
| `python3 plot.py` | 그래프 4장 | `../measurements/plots/*.png` |

---

## 파이프라인

```
IMU 100Hz 6축 스트림
   │  윈도우 L=128 (1.28s), hop=64 (50% 오버랩)          features.py
   ▼
24차원 통계 특징   채널 6 × {mean, std, p2p, mad}
   │  z-score (학습셋 통계, 헤더에 같이 구움)
   ▼
int8 양자화        scale/zero_point, CMSIS-NN s8 규약     quantize.py
   ▼
오토인코더 24-12-4-12-24   int8 곱 / int32 누산 / 정수 재양자화   ae_infer.c
   ▼
재구성 오차 err = Σ(q_out - q_in)²      ← **순수 int32, float 환산 없음**
   ▼
err > 4294 이면 이상
```

### 설계 결정 몇 가지

**특징을 왜 24개 통계값으로?** `timeseries-windowing.md` §4의 권고(통계 특징 먼저, 부족하면
raw 1D-CNN). 넷 다 링버퍼 위에서 싸게 계산된다. 표준편차는 **2-pass**로 짰다 — az는 평균 1g에
표준편차 0.005g라 1-pass 공식 `sqrt(E[x²]-E[x]²)`이 float32에서 자릿수를 다 날린다.

**재구성 오차를 왜 정수로?** 출력층의 (scale, zero_point)를 **입력의 것과 같게 강제**했다.
그러면 `Σ(q_out - q_in)²`이 그대로 의미 있는 값이 되어 MCU에서 float를 한 번도 안 쓰고
임계값과 비교할 수 있다 (`anomaly-detection.md` §5).

**재양자화는 CMSIS-NN 규약 그대로.** 정수 곱 + 시프트(gemmlowp `arm_nn_requantize`와 비트정확
동일). 스트레치로 CMSIS-NN 커널을 끼워 넣을 때 drop-in이 되게 하려고.

---

## 결과 (합성 데이터 기준)

정상 690윈도우로 학습(그중 2세션은 임계값 산정용으로 홀드아웃), 평가 885윈도우(이상 540).

| 모델 | AUC | 정확도 | precision | recall | F1 | FPR |
|---|---|---|---|---|---|---|
| maxabs_z (베이스라인) | 0.9845 | 0.8881 | 0.864 | 0.969 | 0.914 | 0.238 |
| mahalanobis (베이스라인) | 0.9818 | 0.9119 | 0.898 | 0.965 | 0.930 | 0.171 |
| **ae_float32** | **0.9965** | 0.9695 | 0.974 | 0.976 | 0.975 | 0.041 |
| **ae_int8** | **0.9947** | 0.9492 | 0.943 | 0.976 | 0.959 | 0.093 |

### ▶ 양자화 손실: AUC −0.17p, 정확도 −2.03p

이상 종류별 재현율 (int8): bearing 1.000 / imbalance 1.000 / **impact 0.904** / looseness 1.000.
놓치는 건 전부 impact(툭툭 치기)다 — 1.28초 윈도우 안에 임펄스가 안 들어온 윈도우들이라,
윈도우 길이·hop의 문제지 모델 문제가 아니다.

**베이스라인 대조군을 왜 돌리나**: `anomaly-detection.md` §4가 "통계 임계값 → 마할라노비스 →
AE 순서로 올려 왜 굳이 AE인지를 수치로 정당화하라"고 시킨다. 실제로 AE가 이긴다(AUC +1.2~1.5p,
FPR 0.24/0.17 → 0.04). 덤으로 마할라노비스는 Σ⁻¹이 24×24 = **2.3 KB**라 AE(int8 가중치 672 B)
보다 오히려 무겁다.

### 크기

| 항목 | 크기 |
|---|---|
| 가중치 int8 | 672 B |
| bias int32 | 208 B |
| 정규화 상수 float | 192 B |
| **ROM 합계** | **1072 B** |
| 층간 핑퐁 버퍼 (RAM, static) | 48 B |

윈도우 링버퍼(128 × 6 × 4 B = 3 KB)는 펌웨어 쪽 예산이라 여기 안 들어간다.

### 그래프

`../measurements/plots/` — 재구성오차 분포 / ROC(좌상단 확대) / float vs int8 산점도 /
이상종류별 재현율.

`float_vs_int8.png`가 특히 볼 만하다: 정상(파랑)은 대각선에 딱 붙는데, 이상(주황)은 float 오차가
5를 넘는 지점부터 대각선에서 아래로 꺾여 10~30에 눌린다. 입력이 ±127에 **포화**해서 오차가
압축되는 것 — `choose_margin`이 다루는 트레이드오프가 눈에 보이는 그림이다.

---

## C 검증 (`cref/`)

`ae_infer.c` / `ae_infer.h` / `model_weights.h`는 **W3에서 그대로 CubeIDE 프로젝트로 복사**된다.
그래서 malloc 없음, libm은 `sqrtf` 하나, HAL/FreeRTOS/CMSIS 의존 없음 —
그 덕에 PC에서 그대로 컴파일해 검증할 수 있다.

`make -C cref test`가 세 가지를 따로 본다:

| 검사 | 기준 | 결과 |
|---|---|---|
| **정수 경로** — Python q_in → q_out, 재구성오차 | **비트정확** | 0/48 불일치 |
| **float 경로** — 특징, z-score | 특징 상대 2e-4 / z 절대 1e-3 | 최대 1.8e-5, 7.4e-4 |
| **end-to-end** — C 자체 경로의 최종 판정 | 판정 뒤집힘 0 | 0/48 |

특징·정규화는 Python이 float64, C가 float32라 비트 일치가 애초에 불가능하므로 허용오차로 본다.
정수 경로는 변명의 여지가 없으므로 비트정확을 요구한다.

**z-score를 왜 상대오차가 아니라 절대오차로 보나**: `az.mean`은 평균 0.99968 g, 표준편차
0.00056 g다. `z = (x - 0.99968)/0.00056`의 분자가 큰 수끼리의 뺄셈이라 float32 자릿수가 날아가고,
z가 0 근처인 정상 샘플에서 상대오차가 발산한다(기준 자체가 무의미해진다). z는 이미 표준화된 양이라
절대오차가 옳은 기준이다. 실측 최대 절대오차 7.4e-4 = **양자화 LSB의 0.0102배**로, q_in을
흔드는 0.5 LSB 대비 50배 여유. 테스트가 이 LSB 배수를 매번 찍으므로 여유가 줄면 바로 보인다.

---

## 설계에서 실제로 문제였던 것 (전부 수치로 잡힌 것들)

### 1. 조건당 파일 하나 → 장착 기울기로 이상을 맞히는 누수

처음엔 조건마다 긴 CSV 하나씩 뽑았다. 그랬더니 **정상** 테스트 세션의 |z| 최대가 26까지 튀고,
imbalance가 `az.mean` 하나로 분리됐다. `az.mean`은 진동이 아니라 **그날의 장착 기울기**다 —
런 단위 nuisance 상수(기울기·자이로 바이어스·회전수)의 산포가 학습셋에 없어서, 모델이
"이상"이 아니라 "다른 세션"을 학습한 것.

→ 조건당 짧은 세션을 여러 개로 바꿨다. 지금은 정상 |z| 최대 5.35, `az.mean`의 이상 종류별
|z|가 0.63~1.59로 평평하다(= 더 이상 신호가 아니다).

### 2. 윈도우 단위 train/val 분할 → 임계값이 너무 빡빡

임계값을 정상 검증셋 p99로 잡는데 FPR이 **25%**(마할라노비스는 39%)가 나왔다. 규칙상 1%여야 한다.
원인: 학습/검증을 윈도우 단위로 섞어 나눠서, 같은 세션 윈도우가 양쪽에 들어가 있었다. 모델이 그
세션의 nuisance 상수를 외워 검증 오차가 낙관적으로 나오고 → 임계값이 과하게 낮게 잡힌 것.

→ **세션 통째로 홀드아웃**으로 바꿨다. FPR 25% → 4.1%. 정규화 통계도 검증 세션을 빼고 뽑는다.

### 3. `in_margin`을 테스트 AUC로 고르고 있었음

입력 양자화 범위 여유를 후보별 **테스트 AUC**로 골랐는데, 이건 하이퍼파라미터 테스트셋 누수다.
"현장엔 고장 데이터가 없다"는 이 프로젝트의 전제와도 모순된다.

→ 라벨을 안 쓰는 기준으로 교체: *표현범위 반폭이 정상 검증셋 최대 |z|의 2배 이상 되는 가장 작은
margin*. 정상은 절대 포화하지 않으면서 이상 쪽에 여유를 남긴다. 같은 값(3.0)이 선택됐고,
`--sweep`으로 찍어보면 1.5~6.0 구간에서 AUC가 평평해 손해 본 것도 없다.

### 4. numpy와 C의 반올림이 다름

`np.round`는 banker's rounding(짝수 쪽), C `roundf`는 0에서 먼 쪽이다. 정확히 .5인 입력에서
q_in이 갈린다. 추론 경로의 반올림이라 Python을 C에 맞췄다(`quantize._round_half_away`).

---

## 파일

| 파일 | 역할 |
|---|---|
| `config.py` | 전역 상수. **여기만 고치면 헤더까지 따라 바뀐다** |
| `synth_data.py` | 합성 IMU 세션 생성 (보드 실측의 임시 대역) |
| `features.py` | 윈도잉 + 24차원 특징 + 정규화. **C와 1:1 대응되는 레퍼런스** |
| `dataset.py` | manifest 기반 로딩. 윈도우가 세션 경계를 넘지 않게 |
| `autoencoder.py` | 순수 numpy MLP AE (역전파 + Adam 직접 구현) |
| `baseline.py` | maxabs_z / 마할라노비스 대조군, ROC-AUC, 혼동행렬 |
| `train.py` | 학습 + 임계값 산정 (세션 단위 홀드아웃) |
| `quantize.py` | PTQ + **비트정확 정수 순전파 시뮬레이터** |
| `evaluate.py` | float vs int8 → `accuracy.csv` |
| `export_header.py` | → `cref/model_weights.h`, `cref/testvec.bin` |
| `plot.py` | 그래프 4장 |
| `cref/ae_infer.{c,h}` | **펌웨어로 갈 순수 C 추론** |
| `cref/test_ae_int8.c` | Python ↔ C 대조 검증 |

---

## W3에서 할 것

- [ ] 보드 실측 CSV로 교체 → `manifest.csv`에 추가 → `run_all.sh` 재실행
- [ ] `cref/`의 세 파일을 CubeIDE 프로젝트로 복사, 추론 태스크에 연결
- [ ] DWT 사이클로 `ae_run_window()` WCET 측정 (반복 ≥100, 워밍업 제외, 중앙값·p99)
- [ ] 링버퍼 → 윈도우 평탄화 경로 작성 (`ae_features`는 연속 배열을 받는다)
- [ ] impact 재현율 0.904 — 윈도우 길이/hop 재검토할지 결정