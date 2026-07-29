#!/usr/bin/env python3
"""float32 오토인코더 학습 + 임계값 산정 → artifacts/ae_float.npz

파이프라인 1단계. 여기서 나온 npz를 quantize.py가 int8로 바꾸고,
export_header.py가 model_weights.h로 굽는다.

**정상 데이터만 학습에 쓴다** (anomaly-detection.md §1). 이상 세션은 평가에서만 등장하며,
임계값조차 정상 검증셋의 퍼센타일로 정한다 — 이상 샘플로 임계값을 튜닝하면
"고장 데이터를 못 모은다"는 전제를 스스로 깨는 것이고, 새 고장 유형에 일반화가 안 된다.
"""

import argparse

import numpy as np

from autoencoder import MLPAutoencoder
from baseline import (Mahalanobis, MaxAbsZ, confusion, roc_auc,
                      threshold_from_normal)
from config import (ARTIFACT_DIR, BATCH, EPOCHS, LAYER_ACT, LAYER_DIMS, LR,
                    SEED, THRESHOLD_PCT, VAL_FRAC)
from dataset import is_anomaly, load_split
from features import Normalizer


def split_train_val(sessions, val_frac, seed):
    """정상 학습셋을 학습/검증으로 나눈다. 검증은 **임계값 산정 전용**.

    **반드시 세션 단위로 나눈다.** 윈도우 단위로 섞어 나누면 같은 세션의 윈도우가
    학습·검증 양쪽에 들어가고, 모델이 그 세션의 장착 기울기·자이로 바이어스를
    외워버려 검증 오차가 낙관적으로 나온다. 그러면 p99 임계값이 너무 빡빡하게
    잡히고, 처음 보는 정상 세션에서 오탐이 터진다.

    실제로 윈도우 단위 분할로 돌렸을 때 임계값 규칙상 1%여야 할 FPR이 25%가 나왔다.
    (마할라노비스는 39%.) 세션 단위 분할이 이걸 고친다.
    반환: (학습 인덱스, 검증 인덱스)
    """
    uniq = np.unique(sessions)
    rng = np.random.default_rng(seed)
    perm = rng.permutation(len(uniq))
    n_val = max(1, int(round(len(uniq) * val_frac)))
    val_names = set(uniq[perm[:n_val]])
    mask = np.array([s in val_names for s in sessions])
    return ~mask, mask


def main():
    ap = argparse.ArgumentParser(description="AE 학습 + 임계값 산정")
    ap.add_argument("--epochs", type=int, default=EPOCHS)
    ap.add_argument("--seed", type=int, default=SEED)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)

    train = load_split("train")
    test = load_split("test")
    assert not is_anomaly(train.labels).any(), "학습 split에 이상 세션이 섞였다"
    print(f"학습(정상) 윈도우 {len(train.feats)}개 / "
          f"평가 윈도우 {len(test.feats)}개 (이상 {int(is_anomaly(test.labels).sum())}개)")

    # ── 정규화: 학습셋에서만 통계를 뽑는다 ──────────────────────────────
    tr_idx, val_idx = split_train_val(train.sessions, VAL_FRAC, args.seed)
    norm = Normalizer.fit(train.feats[tr_idx])   # 검증 세션은 정규화 통계에도 안 쓴다
    x_tr = norm.transform(train.feats[tr_idx])
    x_val = norm.transform(train.feats[val_idx])
    x_te = norm.transform(test.feats)
    y_te = is_anomaly(test.labels)
    print(f"  학습 {len(x_tr)}윈도우 / 임계값용 검증 {len(x_val)}윈도우 "
          f"({len(np.unique(train.sessions[val_idx]))}개 세션 통째로 홀드아웃)")

    # ── AE 학습 ─────────────────────────────────────────────────────────
    print(f"\nAE {'-'.join(map(str, LAYER_DIMS))}  act={LAYER_ACT}")
    ae = MLPAutoencoder(LAYER_DIMS, LAYER_ACT, seed=args.seed)
    n_param = sum(w.size for w in ae.W) + sum(b.size for b in ae.b)
    print(f"  파라미터 {n_param}개 (int8이면 가중치 "
          f"{sum(w.size for w in ae.W)} B + bias int32 {4 * sum(b.size for b in ae.b)} B)")
    ae.fit(x_tr, epochs=args.epochs, batch=BATCH, lr=LR, seed=args.seed,
           x_val=x_val, log_every=0 if args.quiet else max(1, args.epochs // 6))

    thr = threshold_from_normal(ae.recon_error(x_val), THRESHOLD_PCT)
    print(f"\n임계값(정상 검증 p{THRESHOLD_PCT:g}) = {thr:.6f}")

    # ── 베이스라인 대조 (같은 입력, 같은 임계값 규칙) ───────────────────
    print("\n%-14s %7s %8s %8s %7s %8s" % ("모델", "AUC", "precision", "recall", "F1", "FPR"))
    rows = []
    for model in (MaxAbsZ(), Mahalanobis()):
        model.fit(x_tr)
        t = threshold_from_normal(model.score(x_val), THRESHOLD_PCT)
        s = model.score(x_te)
        c = confusion(s, y_te, t)
        rows.append((model.name, roc_auc(s, y_te), c))
    s_ae = ae.recon_error(x_te)
    rows.append(("ae_float32", roc_auc(s_ae, y_te), confusion(s_ae, y_te, thr)))
    for name, auc, c in rows:
        print("%-14s %7.4f %8.3f %8.3f %7.3f %8.3f"
              % (name, auc, c["precision"], c["recall"], c["f1"], c["fpr"]))

    # 이상 종류별 재현율 — 어느 고장 유형을 놓치는지가 진짜 정보다
    print("\n이상 종류별 AE 재현율:")
    for kind in sorted(set(test.labels[y_te])):
        m = test.labels == kind
        print(f"  {kind:12s} {int((s_ae[m] > thr).sum()):3d}/{int(m.sum()):3d} "
              f"({(s_ae[m] > thr).mean():.3f})")

    out = ARTIFACT_DIR / "ae_float.npz"
    np.savez(out, threshold=np.float64(thr), threshold_pct=np.float64(THRESHOLD_PCT),
             **ae.to_dict(), **norm.to_dict())
    print(f"\n저장: {out.relative_to(ARTIFACT_DIR.parent)}")


if __name__ == "__main__":
    main()
