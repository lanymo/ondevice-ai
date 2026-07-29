#!/usr/bin/env python3
"""artifacts/*.npz → cref/model_weights.h + cref/testvec.bin

**펌웨어가 실제로 컴파일해 넣는 유일한 산출물이 model_weights.h다.** 여기에는
가중치뿐 아니라 윈도우 길이·특징 순서·정규화 상수·양자화 파라미터·임계값이 전부 들어간다
— Python과 C가 서로 다른 상수를 들고 갈라지는 사고를 구조적으로 막기 위해서.

같이 나오는 testvec.bin은 "Python 정수 시뮬 == C 순전파"를 PC에서 비트 단위로 확인하는
데 쓴다(cref/test_ae_int8.c). 보드에 올리기 전에 커널 버그를 여기서 잡는다.

testvec.bin 포맷 (전부 리틀엔디언, 섹션별로 몰아서):
    int32  magic('AETV'=0x56544541), version, n_samples, win_len, n_ch, n_feat
    f32    windows [n_samples][win_len][n_ch]   원시 물리단위 (특징 추출 입력)
    f32    feats   [n_samples][n_feat]          기대 특징
    f32    norms   [n_samples][n_feat]          기대 z-score
    i8     q_in    [n_samples][n_feat]          기대 양자화 입력
    i8     q_out   [n_samples][n_feat]          기대 양자화 출력 (q_in을 넣었을 때)
    i32    err     [n_samples]                  기대 재구성 오차 (정수)
"""

import argparse
import struct

import numpy as np

from autoencoder import MLPAutoencoder
from config import (ARTIFACT_DIR, CHANNELS, CREF_DIR, FEATURE_NAMES, FS_HZ,
                    HOP, LAYER_ACT, LAYER_DIMS, N_CH, N_FEATURES,
                    THRESHOLD_PCT, WIN_LEN)
from dataset import is_anomaly, load_split
from evaluate import choose_margin
from features import Normalizer, feature_names
from quantize import calibrate
from train import split_train_val
from config import SEED, VAL_FRAC
from baseline import threshold_from_normal

MAGIC = 0x56544541
VERSION = 1
N_TESTVEC = 48


def c_array(name, values, ctype, per_line=12, fmt="%d"):
    body = []
    for i in range(0, len(values), per_line):
        body.append("    " + " ".join(fmt % v + "," for v in values[i:i + per_line]))
    return (f"static const {ctype} {name}[{len(values)}] = {{\n"
            + "\n".join(body) + "\n};\n")


def emit_header(qae, norm, thr_int, margin, path):
    L = qae.layers
    n_layers = len(L)
    max_dim = max(LAYER_DIMS)
    w_bytes = sum(l.w_q.size for l in L)
    b_bytes = 4 * sum(l.b_q.size for l in L)
    norm_bytes = 4 * 2 * N_FEATURES

    out = []
    a = out.append
    a("/* 자동 생성 — 손으로 고치지 말 것. `python3 export_header.py` 로 재생성한다.\n"
      " *\n"
      " * ondevice-ai / stm32-edge : int8 오토인코더 이상탐지 모델\n"
      " * 생성기: model/export_header.py   검증: model/cref/test_ae_int8.c\n"
      " *\n"
      f" * 구조    : {'-'.join(map(str, LAYER_DIMS))}  act={LAYER_ACT}\n"
      f" * 입력    : {WIN_LEN}샘플({WIN_LEN / FS_HZ:.2f}s) × {N_CH}채널 → 특징 {N_FEATURES}차원\n"
      f" * 양자화  : 가중치 per-tensor 대칭 / 활성값 비대칭 / 누산 int32 (CMSIS-NN s8 규약)\n"
      " *\n"
      " * ROM 소요:\n"
      f" *   가중치 int8   {w_bytes:6d} B\n"
      f" *   bias   int32  {b_bytes:6d} B\n"
      f" *   정규화 float  {norm_bytes:6d} B\n"
      f" *   합계          {w_bytes + b_bytes + norm_bytes:6d} B\n"
      " */\n")
    a("#ifndef MODEL_WEIGHTS_H\n#define MODEL_WEIGHTS_H\n\n#include <stdint.h>\n\n")

    a("/* ── 데이터 파이프라인 계약 (features.py와 반드시 일치) ────────────── */\n")
    a(f"#define AE_FS_HZ            {FS_HZ}\n")
    a(f"#define AE_WIN_LEN          {WIN_LEN}\n")
    a(f"#define AE_HOP              {HOP}\n")
    a(f"#define AE_N_CH             {N_CH}\n")
    a(f"#define AE_N_FEAT_PER_CH    {len(FEATURE_NAMES)}\n")
    a(f"#define AE_N_FEATURES       {N_FEATURES}\n")
    a(f"#define AE_N_LAYERS         {n_layers}\n")
    a(f"#define AE_MAX_DIM          {max_dim}\n")
    a("/* 채널 순서: " + ", ".join(CHANNELS) + " */\n")
    a("/* 특징 순서(채널 우선): " + ", ".join(f"{c}.{f}" for c in CHANNELS[:1]
                                              for f in FEATURE_NAMES)
      + ", ... */\n\n")

    a("/* ── 이상 판정 ──────────────────────────────────────────────────── */\n")
    a("/* 재구성 오차 err = sum((q_out[i] - q_in[i])^2)  — 순수 int32.\n"
      f" * 출력층 양자화 격자를 입력과 같게 강제했기 때문에 스케일 환산이 필요 없다.\n"
      f" * 임계값은 정상 검증셋 오차의 p{THRESHOLD_PCT:g}. err > 임계값이면 이상. */\n")
    a(f"#define AE_THRESHOLD_INT    {int(thr_int)}\n")
    a(f"#define AE_ERR_MAX          {N_FEATURES * 255 * 255}  /* 이론 최대 — int32 여유 확인용 */\n\n")

    a("/* ── 입력 양자화 (정규화된 특징 → int8) ─────────────────────────── */\n")
    a(f"#define AE_IN_SCALE         {qae.in_scale!r}f\n")
    a(f"#define AE_IN_ZP            {int(qae.in_zp)}\n")
    a(f"/* in_margin={margin} — 정상 검증셋 최대치의 2배까지 표현 (evaluate.py choose_margin) */\n\n")

    a("/* ── 특징 정규화 상수 (학습셋에서 산출) ─────────────────────────── */\n")
    a(c_array("ae_norm_mean", norm.mean, "float", per_line=4, fmt="%.8ef"))
    a("\n")
    a(c_array("ae_norm_std", norm.std, "float", per_line=4, fmt="%.8ef"))
    a("\n")

    names = feature_names()
    for li, l in enumerate(L):
        nout, nin = l.w_q.shape
        a(f"/* ── layer {li}: {nin} → {nout}, act={l.act} "
          f"(w_scale={l.w_scale:.6e}, out_scale={l.out_scale:.6e}, out_zp={l.out_zp}) ── */\n")
        a(c_array(f"ae_w{li}", l.w_q.reshape(-1), "int8_t", per_line=16, fmt="%4d"))
        a(c_array(f"ae_b{li}", l.b_q, "int32_t", per_line=8, fmt="%10d"))
        a("\n")

    a("/* ── 레이어 메타 (인덱스 = 레이어 번호) ─────────────────────────── */\n")
    a(c_array("ae_dim_in", [l.w_q.shape[1] for l in L], "int32_t", per_line=8, fmt="%4d"))
    a(c_array("ae_dim_out", [l.w_q.shape[0] for l in L], "int32_t", per_line=8, fmt="%4d"))
    a(c_array("ae_in_zp", [l.in_zp for l in L], "int32_t", per_line=8, fmt="%4d"))
    a(c_array("ae_out_zp", [l.out_zp for l in L], "int32_t", per_line=8, fmt="%4d"))
    a(c_array("ae_mult", [l.mult for l in L], "int32_t", per_line=4, fmt="%11d"))
    a(c_array("ae_shift", [l.shift for l in L], "int32_t", per_line=8, fmt="%4d"))
    a(c_array("ae_act_min", [l.act_min for l in L], "int32_t", per_line=8, fmt="%4d"))
    a(c_array("ae_act_max", [l.act_max for l in L], "int32_t", per_line=8, fmt="%4d"))
    a("\nstatic const int8_t *const ae_w[AE_N_LAYERS] = { "
      + ", ".join(f"ae_w{i}" for i in range(n_layers)) + " };\n")
    a("static const int32_t *const ae_b[AE_N_LAYERS] = { "
      + ", ".join(f"ae_b{i}" for i in range(n_layers)) + " };\n")
    a("\n#endif /* MODEL_WEIGHTS_H */\n")

    path.write_text("".join(out))
    total = w_bytes + b_bytes + norm_bytes
    print(f"{path}  ({total} B ROM: 가중치 {w_bytes} + bias {b_bytes} + 정규화 {norm_bytes})")
    return total


def emit_testvec(qae, norm, windows, feats, path):
    norms = norm.transform(feats)
    q_in = qae.quantize_input(norms)
    q_out = qae.forward(q_in)
    err = qae.recon_error_int(q_in)
    n = len(feats)
    with open(path, "wb") as f:
        f.write(struct.pack("<6i", MAGIC, VERSION, n, WIN_LEN, N_CH, N_FEATURES))
        np.ascontiguousarray(windows, dtype="<f4").tofile(f)
        np.ascontiguousarray(feats, dtype="<f4").tofile(f)
        np.ascontiguousarray(norms, dtype="<f4").tofile(f)
        np.ascontiguousarray(q_in, dtype=np.int8).tofile(f)
        np.ascontiguousarray(q_out, dtype=np.int8).tofile(f)
        np.ascontiguousarray(err, dtype="<i4").tofile(f)
    print(f"{path}  ({n}개 샘플, {path.stat().st_size} B)")


def main():
    ap = argparse.ArgumentParser(description="model_weights.h + 테스트벡터 생성")
    ap.add_argument("--seed", type=int, default=SEED)
    ap.add_argument("--in-margin", type=float, default=None)
    args = ap.parse_args()

    CREF_DIR.mkdir(parents=True, exist_ok=True)
    d = np.load(ARTIFACT_DIR / "ae_float.npz", allow_pickle=False)
    ae = MLPAutoencoder.from_dict(d)
    norm = Normalizer.from_dict(d)

    train = load_split("train")
    tr_idx, val_idx = split_train_val(train.sessions, VAL_FRAC, args.seed)
    x_tr = norm.transform(train.feats[tr_idx])
    x_val = norm.transform(train.feats[val_idx])

    margin = args.in_margin
    if margin is None:
        margin, _, _ = choose_margin(ae, x_tr, x_val)
    qae = calibrate(ae, x_tr, in_margin=margin)
    thr_int = threshold_from_normal(qae.recon_error_from_float(x_val), THRESHOLD_PCT)
    print(f"in_margin={margin}  정수 임계값={int(round(thr_int))}")

    emit_header(qae, norm, int(round(thr_int)), margin, CREF_DIR / "model_weights.h")

    # 테스트벡터: 정상/이상을 섞어 임계값 양쪽을 다 밟게 한다
    test = load_split("test", with_windows=True)
    y = is_anomaly(test.labels)
    rng = np.random.default_rng(args.seed)
    pick = np.concatenate([
        rng.choice(np.flatnonzero(~y), N_TESTVEC // 2, replace=False),
        rng.choice(np.flatnonzero(y), N_TESTVEC - N_TESTVEC // 2, replace=False)])
    rng.shuffle(pick)
    emit_testvec(qae, norm, test.windows[pick], test.feats[pick],
                 CREF_DIR / "testvec.bin")


if __name__ == "__main__":
    main()
