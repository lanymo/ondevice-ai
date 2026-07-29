#!/usr/bin/env python3
"""축 B 그래프 4장 → measurements/plots/*.png

PLAN.md W4의 "정확도(float vs int8)" 그래프에 해당한다. 축 A 그래프(추론 사이클
히스토그램, config별 액션지연 박스플롯)는 보드 계측이 있어야 하므로 W3 이후.

색은 문서화된 기본 카테고리컬 팔레트를 **공표된 슬롯 순서 그대로** 쓴다(순환 금지).
선/막대는 인접 페어리스트 기준이라 이 순서가 색각 이상 분리도 검증을 통과한 케이스다.
노랑 슬롯은 밝은 배경에서 대비 3:1 미만이라, 막대에는 값을 직접 라벨링해서
색만으로 정보를 전달하지 않게 한다.
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from config import MEAS_DIR, SEED, THRESHOLD_PCT  # noqa: E402
from dataset import is_anomaly  # noqa: E402
from evaluate import choose_margin, evaluate_all  # noqa: E402

# 기본 카테고리컬 팔레트 (슬롯 1~4). 순서 고정, 절대 순환시키지 않는다.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#d8d7d2"
SURFACE = "#fcfcfb"

MODEL_ORDER = ["maxabs_z", "mahalanobis", "ae_float32", "ae_int8"]
COLOR = dict(zip(MODEL_ORDER, SERIES))   # 색은 모델(엔티티)에 붙는다. 순위가 아니라.


def setup_korean_font():
    """한글 라벨용 폰트를 찾는다. 못 찾으면 라벨이 □로 나오므로 조용히 넘어가지 않는다.

    WSL 환경에서는 리눅스 쪽에 한글 폰트가 없어도 Windows 폰트가
    /mnt/c/Windows/Fonts 에 마운트돼 있는 경우가 많아 그걸 주워 쓴다
    (이 개발 환경이 그렇다 — apt로 fonts-nanum을 깔 sudo가 없었다).
    """
    from matplotlib import font_manager as fm

    for cand in ("NanumGothic", "Noto Sans CJK KR", "Malgun Gothic", "AppleGothic"):
        if any(cand == f.name for f in fm.fontManager.ttflist):
            plt.rcParams["font.family"] = cand
            plt.rcParams["axes.unicode_minus"] = False
            return cand

    for path in ("/mnt/c/Windows/Fonts/malgun.ttf",          # WSL → Windows
                 "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"):
        if Path(path).exists():
            fm.fontManager.addfont(path)
            name = fm.FontProperties(fname=path).get_name()
            plt.rcParams["font.family"] = name
            plt.rcParams["axes.unicode_minus"] = False
            print(f"  폰트: {name} ({path})")
            return name

    print("[경고] 한글 폰트가 없다 — 라벨이 □로 나온다. "
          "`sudo apt install fonts-nanum` 후 다시 돌릴 것.")
    return None


def _style(ax, title, xlabel, ylabel):
    ax.set_title(title, color=INK, fontsize=11, loc="left", pad=10)
    ax.set_xlabel(xlabel, color=INK2, fontsize=9)
    ax.set_ylabel(ylabel, color=INK2, fontsize=9)
    ax.tick_params(colors=INK2, labelsize=8, length=0)
    ax.grid(True, color=GRID, linewidth=0.8, alpha=0.7)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)


def _fig(w=7.2, h=4.2):
    fig, ax = plt.subplots(figsize=(w, h), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    return fig, ax


def _save(fig, path):
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)
    print(f"  {path.name}")


def roc_points(scores, y):
    """임계값을 쓸어가며 (FPR, TPR). 외부 의존성 없이 직접 계산."""
    order = np.argsort(-scores, kind="mergesort")
    y_sorted = y[order]
    tp = np.cumsum(y_sorted)
    fp = np.cumsum(~y_sorted)
    tpr = np.concatenate([[0.0], tp / max(int(y.sum()), 1)])
    fpr = np.concatenate([[0.0], fp / max(int((~y).sum()), 1)])
    return fpr, tpr


def plot_error_hist(rows, y, out):
    """재구성 오차 분포: 정상 vs 이상 + 임계값. AE가 무엇을 보고 판단하는지 한 장."""
    r = next(x for x in rows if x["model"] == "ae_int8")
    s, thr = r["_scores"].astype(float), r["threshold"]
    fig, ax = _fig()
    bins = np.logspace(np.log10(max(s.min(), 1)), np.log10(s.max() * 1.05), 45)
    ax.hist(s[~y], bins=bins, color=SERIES[0], alpha=0.85, label=f"정상 (n={int((~y).sum())})")
    ax.hist(s[y], bins=bins, color=SERIES[1], alpha=0.85, label=f"이상 (n={int(y.sum())})")
    ax.axvline(thr, color=INK, linewidth=2, linestyle="--")
    ax.annotate(f"임계값 {int(thr)}\n(정상 검증셋 p{THRESHOLD_PCT:g})",
                xy=(thr, ax.get_ylim()[1] * 0.72), xytext=(8, 0),
                textcoords="offset points", color=INK, fontsize=8.5)
    ax.set_xscale("log")
    _style(ax, "int8 오토인코더 재구성 오차 분포 (정수 영역)",
           "재구성 오차  Σ(q_out - q_in)²  [log]", "윈도우 수")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK2)
    _save(fig, out / "recon_error_hist.png")


def plot_roc(rows, y, out):
    """네 모델 전부 AUC > 0.98이라 [0,1]² 전체를 그리면 곡선이 좌상단 구석에 뭉쳐
    서로 구분이 안 된다. 곡선이 실제로 사는 구간만 확대한다 — 임의추측 대각선은
    이 범위 밖이므로 그리지 않고, 확대 사실을 제목에 밝힌다."""
    fig, ax = _fig(6.8, 4.6)
    curves = {}
    for name in MODEL_ORDER:
        r = next(x for x in rows if x["model"] == name)
        curves[name] = roc_points(r["_scores"].astype(float), y)

    # 재현율 0.9에 도달하는 데 필요한 최대 FPR로 확대 범위를 정한다
    x_hi = 0.0
    for fpr, tpr in curves.values():
        reach = fpr[tpr >= 0.90]
        if len(reach):
            x_hi = max(x_hi, float(reach[0]))
    x_hi = min(1.0, max(0.1, x_hi * 3.0))

    for name in MODEL_ORDER:
        fpr, tpr = curves[name]
        r = next(x for x in rows if x["model"] == name)
        ax.plot(fpr, tpr, color=COLOR[name], linewidth=2,
                label=f"{name}  AUC={r['auc']:.4f}")
    ax.set_xlim(0, x_hi); ax.set_ylim(0.90, 1.005)
    _style(ax, f"ROC — 베이스라인 vs 오토인코더 (좌상단 확대: FPR ≤ {x_hi:.2f})",
           "거짓양성률 (정상을 이상이라 함)", "재현율 (이상을 잡음)")
    ax.legend(frameon=False, fontsize=9, loc="lower right", labelcolor=INK2)
    _save(fig, out / "roc.png")


def plot_float_vs_int8(rows, y, err_scale, out):
    """같은 윈도우에 대한 float 점수 vs int8 점수. 대각선에서 벗어난 만큼이 양자화 손실."""
    f = next(x for x in rows if x["model"] == "ae_float32")["_scores"].astype(float)
    q = next(x for x in rows if x["model"] == "ae_int8")["_scores"].astype(float) * err_scale
    fig, ax = _fig(5.6, 5.2)
    lim = [min(f.min(), q.min()) * 0.8, max(f.max(), q.max()) * 1.2]
    ax.plot(lim, lim, color=GRID, linewidth=1.5, linestyle=":", zorder=1)
    ax.scatter(f[~y], q[~y], s=14, color=SERIES[0], alpha=0.7,
               edgecolors=SURFACE, linewidths=0.5, label="정상", zorder=2)
    ax.scatter(f[y], q[y], s=14, color=SERIES[1], alpha=0.7,
               edgecolors=SURFACE, linewidths=0.5, label="이상", zorder=2)
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlim(lim); ax.set_ylim(lim)
    _style(ax, "양자화 전후 재구성 오차 (같은 윈도우)",
           "float32 재구성 오차 (MSE)", "int8 재구성 오차 (동일 단위 환산)")
    ax.legend(frameon=False, fontsize=9, loc="upper left", labelcolor=INK2)
    _save(fig, out / "float_vs_int8.png")


def plot_recall_by_kind(rows, kinds, out):
    """이상 종류별 재현율. 어느 고장 유형을 놓치는지가 평균 재현율보다 중요한 정보."""
    fig, ax = _fig(7.6, 4.4)
    n = len(MODEL_ORDER)
    width = 0.8 / n
    x = np.arange(len(kinds))
    for i, name in enumerate(MODEL_ORDER):
        r = next(v for v in rows if v["model"] == name)
        vals = [r[f"recall_{k}"] for k in kinds]
        pos = x + (i - (n - 1) / 2) * width
        # 막대 사이 2px 간격을 위해 폭을 살짝 줄인다
        ax.bar(pos, vals, width * 0.92, color=COLOR[name], label=name, zorder=2)
        # 노랑 슬롯이 밝은 배경에서 대비가 낮다 → 값 직접 라벨 (색만으로 전달 금지)
        for p, v in zip(pos, vals):
            ax.text(p, v + 0.015, f"{v:.2f}", ha="center", va="bottom",
                    fontsize=6.5, color=INK2, rotation=90)
    ax.set_xticks(x); ax.set_xticklabels(kinds)
    ax.set_ylim(0, 1.16)
    _style(ax, "이상 종류별 재현율", "", "재현율")
    ax.legend(frameon=False, fontsize=8.5, ncol=4, loc="upper center",
              bbox_to_anchor=(0.5, 1.0), labelcolor=INK2)
    _save(fig, out / "recall_by_kind.png")


def main():
    ap = argparse.ArgumentParser(description="축 B 정확도 그래프")
    ap.add_argument("--in-margin", type=float, default=None)
    ap.add_argument("--seed", type=int, default=SEED)
    args = ap.parse_args()

    setup_korean_font()

    margin = args.in_margin
    if margin is None:
        import numpy as _np
        from autoencoder import MLPAutoencoder
        from config import ARTIFACT_DIR, VAL_FRAC
        from dataset import load_split
        from features import Normalizer
        from train import split_train_val
        d = _np.load(ARTIFACT_DIR / "ae_float.npz", allow_pickle=False)
        ae_ = MLPAutoencoder.from_dict(d)
        nz = Normalizer.from_dict(d)
        tr = load_split("train")
        ti, vi = split_train_val(tr.sessions, VAL_FRAC, args.seed)
        margin, _, _ = choose_margin(ae_, nz.transform(tr.feats[ti]),
                                     nz.transform(tr.feats[vi]))

    rows, qae, (_, _, _, y), test = evaluate_all(margin, args.seed, verbose=False)
    kinds = sorted(set(test.labels[is_anomaly(test.labels)]))

    out = MEAS_DIR / "plots"
    out.mkdir(parents=True, exist_ok=True)
    print(f"그래프 → {out}")
    plot_error_hist(rows, y, out)
    plot_roc(rows, y, out)
    plot_float_vs_int8(rows, y, qae.error_scale(), out)
    plot_recall_by_kind(rows, kinds, out)


if __name__ == "__main__":
    main()
