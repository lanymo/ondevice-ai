"""통계 베이스라인 2종 — "왜 굳이 오토인코더인가"를 수치로 답하기 위한 대조군.

anomaly-detection.md §4가 시킨 순서: 통계적 임계값 → 마할라노비스 → 오토인코더.
AE가 이 둘을 못 이기면 AE를 쓸 이유가 없다. 이 파일은 **AE를 정당화하거나 기각하는**
역할이고, 결과는 measurements/accuracy.csv에 나란히 기록된다.

두 베이스라인 모두 AE와 **완전히 같은 입력**(z-score된 24차원 특징)을 받는다.
그래야 비교가 모델 차이만 반영한다.
"""

import numpy as np


class MaxAbsZ:
    """가장 싼 베이스라인: |z-score|의 최댓값.

    "어떤 특징이라도 정상 범위를 크게 벗어나면 이상". 학습이랄 게 없고
    MCU 비용도 사실상 0(비교 24번). AE가 이걸 못 이기면 게임 끝.
    """

    name = "maxabs_z"

    def fit(self, x_normal):
        return self

    def score(self, x):
        return np.abs(x).max(axis=1)


class Mahalanobis:
    """정상 특징의 평균·공분산에서 얼마나 먼가 (다변량 정규 가정).

    d²(x) = (x-μ)ᵀ Σ⁻¹ (x-μ)

    AE와 비교했을 때의 성격 차이:
      - 선형 상관까지만 본다. 특징 간 **비선형** 관계는 못 잡는다.
      - MCU 비용: Σ⁻¹ 이 24×24 = 576 float(2.3 KB) + 576 MAC.
        AE(724 파라미터, int8이면 724 B)보다 **오히려 무겁다** — 이게 AE 쪽 논거 중 하나.
      - 공분산 추정에 샘플이 차원보다 충분히 많아야 한다(여기선 690 ≫ 24, OK).

    shrinkage: Σ 를 (1-α)Σ + α·diag(Σ) 로 살짝 대각선 쪽으로 당겨 역행렬 불안정을 막는다.
    """

    name = "mahalanobis"

    def __init__(self, shrinkage=0.05):
        self.shrinkage = shrinkage

    def fit(self, x_normal):
        self.mu = x_normal.mean(axis=0)
        cov = np.cov(x_normal, rowvar=False)
        a = self.shrinkage
        cov = (1 - a) * cov + a * np.diag(np.diag(cov))
        self.inv = np.linalg.pinv(cov)
        return self

    def score(self, x):
        d = x - self.mu
        return np.einsum("ij,jk,ik->i", d, self.inv, d)


def threshold_from_normal(scores_normal, pct):
    """정상 검증셋 점수의 상위 퍼센타일 = 임계값 (anomaly-detection.md §3)."""
    return float(np.percentile(scores_normal, pct))


def confusion(scores, is_anom, thr):
    """반환 dict: tp, fn, fp, tn, precision, recall, f1, fpr."""
    pred = scores > thr
    tp = int((pred & is_anom).sum())
    fn = int((~pred & is_anom).sum())
    fp = int((pred & ~is_anom).sum())
    tn = int((~pred & ~is_anom).sum())
    prec = tp / (tp + fp) if tp + fp else 0.0
    rec = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2 * prec * rec / (prec + rec) if prec + rec else 0.0
    fpr = fp / (fp + tn) if fp + tn else 0.0
    return dict(tp=tp, fn=fn, fp=fp, tn=tn, precision=prec, recall=rec, f1=f1, fpr=fpr)


def roc_auc(scores, is_anom):
    """ROC-AUC를 순위(rank)로 계산 — 임계값 선택과 무관한 분리도 지표.

    동점 처리를 위해 평균 순위(tie-averaged rank)를 쓴다. 이상 점수가
    정상 점수보다 클 확률과 같다(Mann-Whitney U).
    """
    n_pos = int(is_anom.sum())
    n_neg = int((~is_anom).sum())
    if n_pos == 0 or n_neg == 0:
        return float("nan")
    order = np.argsort(scores, kind="mergesort")
    ranks = np.empty(len(scores), dtype=np.float64)
    s_sorted = scores[order]
    i = 0
    while i < len(s_sorted):
        j = i
        while j + 1 < len(s_sorted) and s_sorted[j + 1] == s_sorted[i]:
            j += 1
        ranks[order[i:j + 1]] = (i + j) / 2.0 + 1.0
        i = j + 1
    return (ranks[is_anom].sum() - n_pos * (n_pos + 1) / 2.0) / (n_pos * n_neg)
