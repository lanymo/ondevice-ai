"""초소형 MLP 오토인코더 — 순수 numpy (역전파 직접 구현).

**왜 numpy인가**: 24→12→4→12→24, 파라미터 724개짜리 모델에 torch를 끌어오는 건 과하고,
이 리포는 "외부 의존성 최소화"가 규칙이다(CLAUDE.md). 게다가 양자화·C 순전파를
직접 짜야 하므로 순전파 식을 손에 쥐고 있는 편이 낫다.

가중치 레이아웃: W[l] 은 **[out, in]** (출력 뉴런 하나가 한 행).
C 쪽 `for j: for i: acc += w[j][i] * x[i]` 와 메모리 순서가 그대로 일치한다.
export_header.py가 이 순서 그대로 굽는다.
"""

import numpy as np

from config import LAYER_ACT, LAYER_DIMS


def _act(z, kind):
    return np.maximum(z, 0.0) if kind == "relu" else z


def _act_grad(z, kind):
    return (z > 0).astype(z.dtype) if kind == "relu" else np.ones_like(z)


class MLPAutoencoder:
    def __init__(self, dims=LAYER_DIMS, acts=LAYER_ACT, seed=0):
        assert len(acts) == len(dims) - 1
        self.dims, self.acts = tuple(dims), tuple(acts)
        rng = np.random.default_rng(seed)
        self.W, self.b = [], []
        for i, (nin, nout) in enumerate(zip(dims[:-1], dims[1:])):
            # He 초기화 (ReLU 층 기준). 선형층에도 그대로 써도 무방한 규모.
            self.W.append(rng.normal(0.0, np.sqrt(2.0 / nin), (nout, nin)))
            self.b.append(np.zeros(nout))

    # ── 순전파 ──────────────────────────────────────────────────────────
    def forward(self, x, keep=False):
        """x[B, in] → out[B, out]. keep=True면 (out, 캐시) 반환."""
        a = x
        cache = [(None, x)]
        for W, b, act in zip(self.W, self.b, self.acts):
            z = a @ W.T + b
            a = _act(z, act)
            cache.append((z, a))
        return (a, cache) if keep else a

    def encode(self, x):
        """잠재벡터 z까지만 (디버깅/시각화용)."""
        a = x
        mid = len(self.dims) // 2
        for W, b, act in list(zip(self.W, self.b, self.acts))[:mid]:
            a = _act(a @ W.T + b, act)
        return a

    def recon_error(self, x):
        """샘플별 재구성 오차(MSE). anomaly-detection.md §2의 그 값."""
        return ((self.forward(x) - x) ** 2).mean(axis=1)

    # ── 역전파 + Adam ───────────────────────────────────────────────────
    def _backward(self, x, cache):
        B = x.shape[0]
        out = cache[-1][1]
        dA = 2.0 * (out - x) / (B * x.shape[1])
        gW = [None] * len(self.W)
        gb = [None] * len(self.b)
        for l in range(len(self.W) - 1, -1, -1):
            z = cache[l + 1][0]
            a_prev = cache[l][1]
            dZ = dA * _act_grad(z, self.acts[l])
            gW[l] = dZ.T @ a_prev
            gb[l] = dZ.sum(axis=0)
            dA = dZ @ self.W[l]
        return gW, gb

    def fit(self, x, epochs, batch, lr, seed=0, x_val=None, log_every=0):
        """정상 윈도우만 넣는다 (비지도 이상탐지: 정상의 분포만 학습)."""
        rng = np.random.default_rng(seed)
        params = self.W + self.b
        m = [np.zeros_like(p) for p in params]
        v = [np.zeros_like(p) for p in params]
        b1, b2, eps = 0.9, 0.999, 1e-8
        t = 0
        history = []
        n = len(x)
        for ep in range(epochs):
            perm = rng.permutation(n)
            for s in range(0, n, batch):
                xb = x[perm[s:s + batch]]
                _, cache = self.forward(xb, keep=True)
                gW, gb = self._backward(xb, cache)
                grads = gW + gb
                t += 1
                bc1 = 1 - b1 ** t
                bc2 = 1 - b2 ** t
                for i, (p, g) in enumerate(zip(params, grads)):
                    m[i] = b1 * m[i] + (1 - b1) * g
                    v[i] = b2 * v[i] + (1 - b2) * (g * g)
                    p -= lr * (m[i] / bc1) / (np.sqrt(v[i] / bc2) + eps)
            tr = self.recon_error(x).mean()
            va = self.recon_error(x_val).mean() if x_val is not None else np.nan
            history.append((ep, tr, va))
            if log_every and (ep % log_every == 0 or ep == epochs - 1):
                print(f"  epoch {ep:4d}  train_mse={tr:.6f}  val_mse={va:.6f}")
        return history

    # ── 직렬화 ──────────────────────────────────────────────────────────
    def to_dict(self):
        d = {"dims": np.asarray(self.dims), "acts": np.asarray(self.acts)}
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            d[f"W{i}"] = W
            d[f"b{i}"] = b
        return d

    @classmethod
    def from_dict(cls, d):
        dims = tuple(int(v) for v in d["dims"])
        acts = tuple(str(v) for v in d["acts"])
        m = cls(dims, acts)
        m.W = [d[f"W{i}"] for i in range(len(dims) - 1)]
        m.b = [d[f"b{i}"] for i in range(len(dims) - 1)]
        return m