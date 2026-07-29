"""파이프라인 전역 상수 — Python 학습·양자화·export가 모두 여기만 본다.

여기 값을 바꾸면 model_weights.h가 바뀌고, 펌웨어 쪽 링버퍼/아레나 크기도 따라 바뀐다.
export_header.py가 이 상수들을 헤더에 그대로 굽기 때문에 C와 Python이 갈라질 수 없다.
"""

# ── 센서 / 샘플링 ───────────────────────────────────────────────────────────
FS_HZ = 100                      # PLAN.md 확정. MPU6050 100Hz 스트리밍
CHANNELS = ("ax", "ay", "az", "gx", "gy", "gz")
N_CH = len(CHANNELS)

# 물리 단위: 가속도 g, 자이로 dps.
# 주의: 실제 풀스케일 레인지(±2g / ±250dps 여부)는 W2 브링업에서 레지스터 설정을
# 확인한 뒤 확정한다. 여기서는 "CSV가 이미 물리 단위로 변환돼 들어온다"만 가정하며,
# 레인지가 바뀌어도 features.py 아래는 영향 없다(정규화 상수만 재학습).
ACC_UNIT = "g"
GYR_UNIT = "dps"

# ── 윈도잉 (timeseries-windowing.md §3) ─────────────────────────────────────
WIN_LEN = 128                    # 1.28 s. 2의 거듭제곱 → 나중에 radix-2 FFT 특징 확장 여지
HOP = 64                         # 50% 오버랩 → 0.64 s마다 추론
# 감지 지연의 하한 = WIN_LEN / FS_HZ = 1.28 s (축 A 지연 논의와 직결)

# ── 특징 (timeseries-windowing.md §4: 통계 특징 우선) ────────────────────────
# 채널당 4개 × 6채널 = 24차원. 전부 링버퍼 1패스로 계산 가능(누산 4개면 충분):
#   mean : sum
#   std  : sum, sumsq
#   p2p  : min, max
#   mad  : sum|x[n]-x[n-1]|   (거칠기 / 고주파 에너지 proxy)
FEATURE_NAMES = ("mean", "std", "p2p", "mad")
N_FEAT_PER_CH = len(FEATURE_NAMES)
N_FEATURES = N_CH * N_FEAT_PER_CH          # 24

# ── 오토인코더 구조 (anomaly-detection.md §2) ───────────────────────────────
# 24 → 12 → 4 → 12 → 24. 병목 4차원.
# 파라미터 수 = (24·12+12) + (12·4+4) + (4·12+12) + (12·24+24) = 724
LAYER_DIMS = (N_FEATURES, 12, 4, 12, N_FEATURES)
# 활성함수: 은닉 ReLU, 잠재(z)와 출력은 선형
LAYER_ACT = ("relu", "linear", "relu", "linear")

# ── 학습 하이퍼파라미터 ─────────────────────────────────────────────────────
SEED = 20260728
EPOCHS = 600
BATCH = 32
LR = 3e-3
VAL_FRAC = 0.25                  # 정상 데이터 중 검증(임계값 산정)용 비율

# ── 임계값 (anomaly-detection.md §3) ────────────────────────────────────────
THRESHOLD_PCT = 99.0             # 정상 검증셋 재구성오차의 p99

# ── 양자화 (quantization-basics.md §2·§3) ───────────────────────────────────
# 가중치: per-tensor 대칭 (zero_point = 0)
# 활성값: 비대칭 (scale, zero_point) — CMSIS-NN *_s8 규약과 동일
# 재양자화: int32 곱 + 시프트 (gemmlowp/arm_nn_requantize 동일 semantics)
CALIB_PCT = 99.9                 # 활성값 range 산정 시 퍼센타일 클리핑
INT8_MIN, INT8_MAX = -128, 127

# ── 경로 ────────────────────────────────────────────────────────────────────
from pathlib import Path  # noqa: E402

MODEL_DIR = Path(__file__).resolve().parent
EDGE_DIR = MODEL_DIR.parent
DATA_DIR = MODEL_DIR / "data"            # CSV (합성 or 보드 실측)
ARTIFACT_DIR = MODEL_DIR / "artifacts"   # npz, 임계값, 테스트벡터
CREF_DIR = MODEL_DIR / "cref"            # 순수 C 순전파 + 생성된 model_weights.h
MEAS_DIR = EDGE_DIR / "measurements"     # accuracy.csv 등