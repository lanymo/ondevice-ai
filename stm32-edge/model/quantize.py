"""PTQ(사후 양자화) + **비트정확 정수 순전파 시뮬레이터**.

quantization-basics.md §2·§3·§4를 그대로 구현한다. 규약은 CMSIS-NN `arm_fully_connected_s8`
과 동일하게 맞췄다 — 나중에 스트레치로 CMSIS-NN을 끼워 넣을 때 drop-in이 되도록.

  가중치 : per-tensor **대칭** (zero_point = 0), scale = max|W| / 127
  활성값 : **비대칭** (scale, zero_point)  — ReLU 출력은 한쪽으로 쏠려 있어 비대칭이 이득
  누산   : int32 (24항 × 255 × 127 ≈ 7.8e5 → int32 여유 충분)
  재양자화: int32 곱 + 시프트. float 나눗셈 없음 (quantization-basics.md §6-5)

한 레이어의 정수 순전파:

    acc = bias_q[j]                                  (int32)
    for i: acc += (q_in[i] - in_zp) * w_q[j][i]      (int8 × int8 → int32 누산)
    acc = requantize(acc, mult[j], shift[j])
    acc += out_zp
    q_out[j] = clamp(acc, act_min, act_max)          (ReLU는 clamp로 흡수)

**출력층만 특별 취급**: 출력층의 (scale, zero_point)를 **입력의 것과 같게 강제**한다.
그러면 재구성 오차를 스케일 환산 없이 순수 정수로 계산할 수 있다:

    err_q = Σ (q_out[i] - q_in[i])²      (int32, 최대 24·255² = 1.56e6)

anomaly-detection.md §5의 "재구성 오차 임계 비교는 정수 영역에서 처리 가능(스케일 주의)"가
이 얘기다. MCU에서 float 한 번도 안 쓰고 임계값과 비교할 수 있다.
"""

import numpy as np

from config import CALIB_PCT, INT8_MAX, INT8_MIN

# ── 고정소수점 유틸 — gemmlowp / arm_nn_requantize 와 **비트정확 동일** ──────
# C 쪽 cref/ae_int8.c 에 같은 함수가 있고, 둘이 일치하는지 test_ae_int8이 검증한다.


def quantize_multiplier(real_mult):
    """실수 배율 → (int32 mult ∈ [2^30, 2^31), shift). real_mult > 0 가정."""
    if real_mult <= 0:
        raise ValueError(f"real_mult must be > 0, got {real_mult}")
    frac, exp = np.frexp(real_mult)          # real_mult = frac · 2^exp, frac ∈ [0.5, 1)
    q = int(round(frac * (1 << 31)))
    if q == (1 << 31):                        # frac이 1에 붙어 반올림된 경우
        q //= 2
        exp += 1
    assert (1 << 30) <= q < (1 << 31), q
    return q, int(exp)


def _sat_doubling_high_mul(a, b):
    """round(a·b / 2^31), int32 포화. a는 배열, b는 스칼라."""
    a = np.asarray(a, dtype=np.int64)
    ab = a * np.int64(b)
    nudge = np.where(ab >= 0, np.int64(1) << 30, np.int64(1) - (np.int64(1) << 30))
    num = ab + nudge
    # C의 / 는 0 방향 절삭인데 numpy의 >> 는 음의 무한대 방향이다. 명시적으로 0 방향으로 맞춘다.
    # (float 나눗셈은 num이 최대 ~2^62라 float64 유효자릿수를 넘어 못 쓴다.)
    out = np.where(num >= 0, num >> 31, -((-num) >> 31))
    return np.clip(out, -(1 << 31), (1 << 31) - 1).astype(np.int64)


def _rounding_divide_by_pot(x, exponent):
    """round(x / 2^exponent), 절반은 0에서 먼 쪽으로. exponent ≥ 0."""
    if exponent == 0:
        return x
    x = np.asarray(x, dtype=np.int64)
    mask = (np.int64(1) << exponent) - 1
    remainder = x & mask
    threshold = (mask >> 1) + (x < 0).astype(np.int64)
    return (x >> exponent) + (remainder > threshold).astype(np.int64)


def requantize(v, mult, shift):
    """arm_nn_requantize 와 동일. shift > 0이면 왼쪽 시프트, ≤ 0이면 반올림 오른쪽 시프트."""
    left = shift if shift > 0 else 0
    right = 0 if shift > 0 else -shift
    return _rounding_divide_by_pot(
        _sat_doubling_high_mul(np.asarray(v, dtype=np.int64) << left, mult), right)


# ── 텐서 양자화 파라미터 산정 ───────────────────────────────────────────────

def affine_params(vmin, vmax):
    """비대칭 int8 파라미터. 실수 0이 반드시 표현 가능하도록 범위에 0을 포함시킨다."""
    vmin = min(float(vmin), 0.0)
    vmax = max(float(vmax), 0.0)
    if vmax - vmin < 1e-12:
        vmax = vmin + 1e-12
    scale = (vmax - vmin) / (INT8_MAX - INT8_MIN)
    zp = int(round(INT8_MIN - vmin / scale))
    return scale, int(np.clip(zp, INT8_MIN, INT8_MAX))


def _round_half_away(x):
    """C의 roundf()와 동일한 반올림 (0에서 먼 쪽).

    np.round는 banker's rounding(짝수 쪽으로)이라 정확히 .5인 값에서 C와 갈린다.
    추론 경로에 쓰이는 반올림은 C와 비트 단위로 같아야 하므로 여기서 맞춘다.
    (가중치·bias 양자화는 오프라인이라 상관없지만, 입력 양자화는 매 추론 경로다.)
    """
    x = np.asarray(x, dtype=np.float64)
    return np.sign(x) * np.floor(np.abs(x) + 0.5)


def quantize_tensor(x, scale, zp):
    return np.clip(_round_half_away(np.asarray(x) / scale) + zp,
                   INT8_MIN, INT8_MAX).astype(np.int32)


def dequantize_tensor(q, scale, zp):
    return (np.asarray(q, dtype=np.float64) - zp) * scale


# ── 양자화된 모델 ───────────────────────────────────────────────────────────

class QuantLayer:
    __slots__ = ("w_q", "b_q", "w_scale", "in_scale", "in_zp", "out_scale",
                 "out_zp", "mult", "shift", "act_min", "act_max", "act")

    def forward(self, q_in):
        """q_in [B, in] int → q_out [B, out] int. C 코드와 비트정확히 같아야 한다."""
        acc = (np.asarray(q_in, dtype=np.int64) - self.in_zp) @ self.w_q.T.astype(np.int64)
        acc = acc + self.b_q.astype(np.int64)
        if np.abs(acc).max(initial=0) >= (1 << 31):
            raise OverflowError("int32 누산기 오버플로 — 스케일 재검토 필요")
        acc = requantize(acc, self.mult, self.shift) + self.out_zp
        return np.clip(acc, self.act_min, self.act_max).astype(np.int32)


class QuantAutoencoder:
    """int8 오토인코더. forward는 MCU가 하는 연산을 비트 단위로 흉내낸다."""

    def __init__(self, layers, in_scale, in_zp):
        self.layers = layers
        self.in_scale = in_scale
        self.in_zp = in_zp

    def quantize_input(self, x_norm):
        return quantize_tensor(x_norm, self.in_scale, self.in_zp)

    def forward(self, q_in):
        q = np.asarray(q_in, dtype=np.int32)
        for lyr in self.layers:
            q = lyr.forward(q)
        return q

    def recon_error_int(self, q_in):
        """Σ(q_out - q_in)² — 정수 그대로. 임계값도 같은 단위로 비교한다."""
        d = self.forward(q_in).astype(np.int64) - np.asarray(q_in, dtype=np.int64)
        return (d * d).sum(axis=1)

    def recon_error_from_float(self, x_norm):
        return self.recon_error_int(self.quantize_input(x_norm))

    def error_scale(self):
        """정수 오차 → float MSE 환산 계수. float_mse ≈ err_q · s² / n_feat."""
        n = self.layers[-1].w_q.shape[0]
        return self.in_scale ** 2 / n


def calibrate(model, x_calib, pct=CALIB_PCT, in_margin=1.0):
    """대표 입력을 흘려 각 텐서의 범위를 수집 → 양자화 파라미터 산정.

    x_calib 는 **정상** 데이터여야 한다. 현장에서 고장 데이터를 못 모은다는 전제와
    일관되게, 캘리브레이션도 정상만 쓴다.

    in_margin: 입력 범위에 곱하는 여유. 이상 입력은 정상 범위 밖으로 나가므로
    margin=1.0이면 포화되어 재구성 오차가 압축된다. 1보다 크게 주면 이상 쪽
    해상도를 확보하는 대신 정상 쪽 해상도를 잃는다 — 정확도로 고를 것.
    """
    lo = lambda a: np.percentile(a, 100 - pct)   # noqa: E731
    hi = lambda a: np.percentile(a, pct)         # noqa: E731

    in_scale, in_zp = affine_params(lo(x_calib) * in_margin, hi(x_calib) * in_margin)

    # float 순전파로 각 층 출력의 실제 분포를 본다
    _, cache = model.forward(x_calib, keep=True)
    acts = [a for _, a in cache[1:]]

    layers = []
    cur_scale, cur_zp = in_scale, in_zp
    n_layers = len(model.W)
    for li, (W, b, kind, a_out) in enumerate(zip(model.W, model.b, model.acts, acts)):
        w_scale = float(np.abs(W).max()) / INT8_MAX
        if w_scale <= 0:
            raise ValueError(f"layer {li}: 가중치가 전부 0")
        w_q = np.clip(np.round(W / w_scale), INT8_MIN, INT8_MAX).astype(np.int32)

        if li == n_layers - 1:
            # 출력층: 입력과 같은 양자화 격자를 쓴다 → 오차를 정수로 계산 가능
            out_scale, out_zp = in_scale, in_zp
        elif kind == "relu":
            out_scale, out_zp = affine_params(0.0, hi(a_out))
        else:
            out_scale, out_zp = affine_params(lo(a_out), hi(a_out))

        lyr = QuantLayer()
        lyr.w_q, lyr.w_scale = w_q, w_scale
        lyr.in_scale, lyr.in_zp = cur_scale, cur_zp
        lyr.out_scale, lyr.out_zp = out_scale, out_zp
        lyr.act = kind
        lyr.b_q = np.round(np.asarray(b) / (cur_scale * w_scale)).astype(np.int64)
        if np.abs(lyr.b_q).max(initial=0) >= (1 << 31):
            raise OverflowError(f"layer {li}: bias가 int32를 넘는다")
        lyr.b_q = lyr.b_q.astype(np.int32)
        lyr.mult, lyr.shift = quantize_multiplier(cur_scale * w_scale / out_scale)
        # ReLU는 별도 연산이 아니라 clamp 하한으로 흡수된다 (실수 0 = 정수 out_zp)
        lyr.act_min = out_zp if kind == "relu" else INT8_MIN
        lyr.act_max = INT8_MAX
        layers.append(lyr)
        cur_scale, cur_zp = out_scale, out_zp

    return QuantAutoencoder(layers, in_scale, in_zp)