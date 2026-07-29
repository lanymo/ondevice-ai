#!/usr/bin/env python3
"""float32 vs int8 정확도 비교 → measurements/accuracy.csv + artifacts/int8_model.npz

PLAN.md §4 측정표의 "엣지 기능 정확도 (float vs int8)" 행이 이 스크립트의 산출물이다.
축 B의 핵심 수치: **양자화했더니 정확도가 이만큼만 떨어졌다.**

임계값 처리에서 한 가지 주의: float 임계값을 그대로 정수 영역에 쓸 수 없다(단위가 다르다).
int8 모델은 **자기 자신의 정상 검증셋 오차 분포에서 p99를 다시 잡는다.** 이게 실제 배포
절차와도 같다 — 보드에 올릴 모델로 임계값을 정한다.
"""

import argparse
import csv

import numpy as np

from autoencoder import MLPAutoencoder
from baseline import (Mahalanobis, MaxAbsZ, confusion, roc_auc,
                      threshold_from_normal)
from config import ARTIFACT_DIR, MEAS_DIR, SEED, THRESHOLD_PCT, VAL_FRAC
from dataset import is_anomaly, load_split
from features import Normalizer
from quantize import calibrate
from train import split_train_val


def evaluate_all(in_margin, seed=SEED, verbose=True):
    d = np.load(ARTIFACT_DIR / "ae_float.npz", allow_pickle=False)
    ae = MLPAutoencoder.from_dict(d)
    norm = Normalizer.from_dict(d)

    train = load_split("train")
    test = load_split("test")
    tr_idx, val_idx = split_train_val(train.sessions, VAL_FRAC, seed)
    x_tr = norm.transform(train.feats[tr_idx])
    x_val = norm.transform(train.feats[val_idx])
    x_te = norm.transform(test.feats)
    y_te = is_anomaly(test.labels)

    qae = calibrate(ae, x_tr, in_margin=in_margin)

    results = []   # (model, threshold, scores_test)

    for m in (MaxAbsZ(), Mahalanobis()):
        m.fit(x_tr)
        results.append((m.name, threshold_from_normal(m.score(x_val), THRESHOLD_PCT),
                        m.score(x_te)))

    thr_f = threshold_from_normal(ae.recon_error(x_val), THRESHOLD_PCT)
    results.append(("ae_float32", thr_f, ae.recon_error(x_te)))

    thr_q = threshold_from_normal(qae.recon_error_from_float(x_val), THRESHOLD_PCT)
    results.append(("ae_int8", thr_q, qae.recon_error_from_float(x_te)))

    rows = []
    for name, thr, s in results:
        c = confusion(s, y_te, thr)
        c.update(model=name, auc=roc_auc(s, y_te), threshold=thr,
                 accuracy=(c["tp"] + c["tn"]) / len(y_te))
        c["_scores"] = s          # 그래프용. '_' 접두어라 CSV 컬럼에는 안 나간다.
        # 이상 종류별 재현율
        for kind in sorted(set(test.labels[y_te])):
            m_ = test.labels == kind
            c[f"recall_{kind}"] = float((s[m_] > thr).mean())
        rows.append(c)

    if verbose:
        _print_table(rows, qae, ae, x_te, y_te, test, results)
    return rows, qae, (x_tr, x_val, x_te, y_te), test


def _print_table(rows, qae, ae, x_te, y_te, test, results):
    kinds = sorted(set(test.labels[y_te]))
    hdr = "%-13s %7s %8s %8s %7s %7s %7s" % (
        "모델", "AUC", "정확도", "precision", "recall", "F1", "FPR")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print("%-13s %7.4f %8.4f %8.3f %7.3f %7.3f %7.3f"
              % (r["model"], r["auc"], r["accuracy"], r["precision"],
                 r["recall"], r["f1"], r["fpr"]))

    print("\n이상 종류별 재현율:")
    print("%-13s %s" % ("모델", " ".join("%11s" % k for k in kinds)))
    for r in rows:
        print("%-13s %s" % (r["model"], " ".join("%11.3f" % r[f"recall_{k}"] for k in kinds)))

    f = next(r for r in rows if r["model"] == "ae_float32")
    q = next(r for r in rows if r["model"] == "ae_int8")
    print(f"\n▶ 양자화 손실: AUC {f['auc']:.4f} → {q['auc']:.4f} "
          f"({(q['auc'] - f['auc']) * 100:+.2f}p),  "
          f"정확도 {f['accuracy']:.4f} → {q['accuracy']:.4f} "
          f"({(q['accuracy'] - f['accuracy']) * 100:+.2f}p)")

    # 두 모델이 실제로 얼마나 다른 점수를 내는가 (순위상관 아닌 절대 비교)
    s_f = ae.recon_error(x_te)
    s_q = qae.recon_error_from_float(x_te).astype(np.float64) * qae.error_scale()
    rel = np.abs(s_q - s_f) / np.maximum(s_f, 1e-9)
    print(f"  재구성오차 상대오차: 중앙값 {np.median(rel):.4f}  p99 {np.percentile(rel, 99):.4f}")

    sat = (np.abs(qae.quantize_input(x_te)) == 127).mean()
    print(f"  입력 포화 비율(|q|=127): {sat:.4f}")


def write_csv(rows, path, extra):
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = ["model", "auc", "accuracy", "precision", "recall", "f1", "fpr",
            "tp", "fn", "fp", "tn", "threshold"]
    keys += [k for k in rows[0] if k.startswith("recall_")]
    keys += list(extra)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({**{k: r.get(k, "") for k in keys}, **extra})
    print(f"\n저장: {path}")


MARGIN_CANDIDATES = (1.0, 1.5, 2.0, 3.0, 4.0, 6.0)
COVER = 2.0   # 입력 표현범위가 정상 검증셋 최대치의 몇 배까지 닿아야 하는가


def choose_margin(ae, x_tr, x_val, candidates=MARGIN_CANDIDATES, cover=COVER):
    """in_margin을 **라벨 없이** 고른다.

    이상 라벨로 AUC를 보고 고르면 테스트셋 누수다(현장에선 고장 데이터가 없다는
    전제와도 모순). 대신 정상 데이터만으로 판단한다:

      - margin이 작으면 이상 입력이 ±127에 포화 → 재구성 오차가 압축돼 탐지가 무뎌진다
      - margin이 크면 정상 구간의 양자화 해상도가 낮아진다

    기준: **표현 범위 반폭이 정상 검증셋 최대 |z|의 cover배 이상 되는 가장 작은 margin.**
    정상은 절대 포화하지 않으면서(→ 임계값 산정이 왜곡되지 않음) 이상 쪽으로 cover배의
    여유를 남긴다. 라벨을 본 사후 확인은 --sweep으로 따로 찍을 수 있다.
    """
    need = cover * float(np.abs(x_val).max())
    for m in candidates:
        half = 127.5 * calibrate(ae, x_tr, in_margin=m).in_scale
        if half >= need:
            return m, half, need
    return candidates[-1], 127.5 * calibrate(ae, x_tr, in_margin=candidates[-1]).in_scale, need


def main():
    ap = argparse.ArgumentParser(description="float vs int8 정확도")
    ap.add_argument("--in-margin", type=float, default=None,
                    help="입력 양자화 범위 여유. 미지정 시 라벨 없이 자동 선택")
    ap.add_argument("--sweep", action="store_true",
                    help="margin별 int8 성능을 찍는다 (사후 진단용, 선택에는 쓰지 않음)")
    ap.add_argument("--seed", type=int, default=SEED)
    args = ap.parse_args()

    margin = args.in_margin
    if margin is None:
        d = np.load(ARTIFACT_DIR / "ae_float.npz", allow_pickle=False)
        ae = MLPAutoencoder.from_dict(d)
        norm = Normalizer.from_dict(d)
        train = load_split("train")
        tr_idx, val_idx = split_train_val(train.sessions, VAL_FRAC, args.seed)
        margin, half, need = choose_margin(ae, norm.transform(train.feats[tr_idx]),
                                           norm.transform(train.feats[val_idx]))
        print(f"in_margin 자동 선택(라벨 미사용): {margin} "
              f"→ 표현범위 ±{half:.2f} (필요 ±{need:.2f})\n")

    if args.sweep:
        print("[사후 진단] margin별 int8 성능 — 선택 근거로 쓰지 않는다\n")
        print("%8s %10s %10s %12s" % ("margin", "int8 AUC", "int8 F1", "테스트 포화율"))
        for m in MARGIN_CANDIDATES:
            rows, qae, (_, _, x_te, _), _ = evaluate_all(m, args.seed, verbose=False)
            r = next(x for x in rows if x["model"] == "ae_int8")
            sat = (np.abs(qae.quantize_input(x_te)) == 127).mean()
            print("%8.1f %10.4f %10.4f %12.4f%s"
                  % (m, r["auc"], r["f1"], sat, "  ← 선택" if m == margin else ""))
        print()

    rows, qae, _, _ = evaluate_all(margin, args.seed, verbose=True)
    write_csv(rows, MEAS_DIR / "accuracy.csv",
              extra={"in_margin": margin, "threshold_pct": THRESHOLD_PCT,
                     "data_source": "synthetic_proxy"})

    np.savez(ARTIFACT_DIR / "int8_model.npz",
             in_scale=qae.in_scale, in_zp=qae.in_zp, in_margin=margin,
             threshold_int=np.int64(next(r for r in rows if r["model"] == "ae_int8")["threshold"]),
             **{f"{k}{i}": getattr(l, k) for i, l in enumerate(qae.layers)
                for k in ("w_q", "b_q", "mult", "shift", "act_min", "act_max",
                          "out_scale", "out_zp", "w_scale")})
    print(f"저장: {ARTIFACT_DIR / 'int8_model.npz'}")


if __name__ == "__main__":
    main()