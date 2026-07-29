"""윈도잉 + 특징 추출 — **펌웨어 C 코드와 1:1로 대응되게** 짠 레퍼런스.

timeseries-windowing.md §3·§4 를 그대로 구현한다. 여기서 정한 특징 순서/공식이
model_weights.h 의 정규화 상수와 펌웨어 feature 계산 코드의 계약(contract)이다.

특징 벡터 레이아웃 (24차원, **채널 우선**):
    [ch0.mean, ch0.std, ch0.p2p, ch0.mad,  ch1.mean, ... , ch5.mad]
    ch 순서 = config.CHANNELS = (ax, ay, az, gx, gy, gz)

각 특징의 정의 (N = WIN_LEN):
    mean = Σx / N
    std  = sqrt( Σ(x-mean)² / N )          ← 2-pass. 이유는 아래 주석
    p2p  = max(x) - min(x)
    mad  = Σ|x[n] - x[n-1]| / (N-1)        ← 거칠기/고주파 에너지 proxy

**왜 2-pass std인가**: 1-pass 공식 sqrt(Σx²/N - mean²)은 az처럼 평균이 1g이고
표준편차가 0.005g인 신호에서 큰 수끼리 빼면서 유효숫자가 날아간다(catastrophic
cancellation). float32에서 1.0의 상대오차 ~1e-7 → 절대오차 1e-7이 참값 2.5e-5에
비해 무시 못 할 크기. 링버퍼엔 윈도우 전체가 이미 있으므로 2-pass 비용(128샘플 ×
6채널 = 768 flop 추가)은 무시할 만하다. 펌웨어도 반드시 2-pass로 짠다.
"""

import numpy as np

from config import (CHANNELS, FEATURE_NAMES, HOP, N_CH, N_FEAT_PER_CH,
                    N_FEATURES, WIN_LEN)


def feature_names():
    """24개 특징의 사람이 읽는 이름 — 순서는 C 레이아웃과 동일."""
    return [f"{ch}.{fn}" for ch in CHANNELS for fn in FEATURE_NAMES]


def load_run(path):
    """CSV 한 개를 읽는다. 반환 data[n, 6] (float64, 물리 단위).

    합성 데이터든 보드 UART 덤프든 같은 함수로 읽힌다 — '#' 주석과 헤더 줄을 건너뛴다.
    """
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line[0].isalpha():          # 컬럼 헤더 줄
                continue
            parts = line.split(",")
            rows.append([float(v) for v in parts[1:1 + N_CH]])   # t_ms 버림
    if not rows:
        raise ValueError(f"{path}: 샘플이 없다")
    return np.asarray(rows, dtype=np.float64)


def make_windows(data, win_len=WIN_LEN, hop=HOP):
    """스트림을 겹치는 윈도우로 자른다. 반환 [n_win, win_len, n_ch].

    꼬리에 win_len 미만이 남으면 버린다 — MCU도 윈도우가 다 차야 추론하므로 동일.
    """
    n = len(data)
    if n < win_len:
        return np.empty((0, win_len, data.shape[1]))
    starts = range(0, n - win_len + 1, hop)
    return np.stack([data[s:s + win_len] for s in starts])


def extract_features(windows):
    """[n_win, win_len, n_ch] → [n_win, 24]. 공식은 모듈 docstring 참고."""
    if windows.shape[0] == 0:
        return np.empty((0, N_FEATURES))
    n_win, win_len, n_ch = windows.shape
    assert n_ch == N_CH, f"채널 수 불일치: {n_ch} != {N_CH}"

    mean = windows.mean(axis=1)                                    # [w, ch]
    centered = windows - mean[:, None, :]
    std = np.sqrt((centered ** 2).sum(axis=1) / win_len)           # 2-pass
    p2p = windows.max(axis=1) - windows.min(axis=1)
    mad = np.abs(np.diff(windows, axis=1)).sum(axis=1) / (win_len - 1)

    # [w, ch, 4] → [w, 24] (채널 우선 평탄화)
    stacked = np.stack([mean, std, p2p, mad], axis=2)
    assert stacked.shape == (n_win, n_ch, N_FEAT_PER_CH)
    return stacked.reshape(n_win, N_FEATURES)


def features_from_csv(path, win_len=WIN_LEN, hop=HOP):
    """CSV 한 개 → 특징행렬 [n_win, 24]."""
    return extract_features(make_windows(load_run(path), win_len, hop))


class Normalizer:
    """채널별이 아니라 **특징별** z-score (timeseries-windowing.md §5).

    학습셋(정상)에서 구한 mean/std를 추론에도 똑같이 써야 하므로 이 상수들도
    model_weights.h에 함께 굽는다.
    """

    def __init__(self, mean, std):
        self.mean = np.asarray(mean, dtype=np.float64)
        self.std = np.asarray(std, dtype=np.float64)

    @classmethod
    def fit(cls, feats):
        mean = feats.mean(axis=0)
        std = feats.std(axis=0)
        # 상수 특징(std=0)에서 0으로 나누는 걸 막는다. 정규화 후 그냥 0이 된다.
        std = np.where(std < 1e-9, 1.0, std)
        return cls(mean, std)

    def transform(self, feats):
        return (feats - self.mean) / self.std

    def to_dict(self):
        return {"norm_mean": self.mean, "norm_std": self.std}

    @classmethod
    def from_dict(cls, d):
        return cls(d["norm_mean"], d["norm_std"])