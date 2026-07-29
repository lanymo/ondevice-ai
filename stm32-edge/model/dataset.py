"""manifest.csv를 읽어 특징행렬로 바꾸는 계층.

manifest.csv 한 줄 = 세션 하나(= CSV 하나):

    file,split,label
    normal_train_00.csv,train,normal
    anom_impact_01.csv,test,impact

**윈도우는 세션 경계를 넘지 않는다** — 센서를 떼었다 붙인 두 구간을 한 윈도우에
섞으면 존재하지 않는 신호를 만들어낸다. 세션별로 잘라서 이어붙인다.

합성이든 보드 실측이든 이 계층 위로는 구분이 없다. 실측 CSV를 data/에 넣고
manifest에 줄만 추가하면 학습·평가가 그대로 돌아간다.
"""

from collections import namedtuple

import numpy as np

from config import DATA_DIR, HOP, WIN_LEN
from features import extract_features, load_run, make_windows

Split = namedtuple("Split", "feats labels sessions windows")
#   feats    [n_win, 24]
#   labels   [n_win]  문자열 ('normal', 'imbalance', ...)
#   sessions [n_win]  각 윈도우가 온 세션 파일명 (누수 진단용)
#   windows  [n_win, WIN_LEN, 6] 또는 None (with_windows=True일 때만 — C 테스트벡터용)


def read_manifest(path=None):
    path = path or (DATA_DIR / "manifest.csv")
    if not path.exists():
        raise FileNotFoundError(
            f"{path} 없음. 먼저 `python3 synth_data.py`를 돌리거나 "
            "실측 CSV + manifest를 만들어라.")
    rows = []
    for i, line in enumerate(path.read_text().splitlines()):
        line = line.strip()
        if not line or line.startswith("#") or i == 0:
            continue
        fname, split, label = (c.strip() for c in line.split(","))
        rows.append((fname, split, label))
    return rows


def load_split(split_name, manifest_path=None, win_len=WIN_LEN, hop=HOP,
               with_windows=False):
    feats, labels, sessions, wins = [], [], [], []
    for fname, split, label in read_manifest(manifest_path):
        if split != split_name:
            continue
        w = make_windows(load_run(DATA_DIR / fname), win_len, hop)
        if len(w) == 0:
            raise ValueError(f"{fname}: 윈도우 길이 {win_len}보다 짧다")
        feats.append(extract_features(w))
        labels.extend([label] * len(w))
        sessions.extend([fname] * len(w))
        if with_windows:
            wins.append(w)
    if not feats:
        raise ValueError(f"manifest에 split={split_name} 세션이 없다")
    return Split(np.vstack(feats), np.asarray(labels), np.asarray(sessions),
                 np.concatenate(wins) if with_windows else None)


def is_anomaly(labels):
    """라벨 배열 → bool 배열 (normal=False). 이상 종류는 여러 개여도 전부 True."""
    return labels != "normal"
