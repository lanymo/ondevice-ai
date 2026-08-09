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
    f32    reconf  [n_samples][n_feat]          기대 float32 재구성 (v2~)
    f32    errf    [n_samples]                  기대 float32 재구성 오차 = MSE (v2~)

v2에서 뒤의 두 섹션이 붙었다 — float32 대조군 커널(cref/ae_infer_f32.c)도 검증 없이
보드에 올리면 "느린 커널"과 "틀린 커널"을 구분할 수 없기 때문이다. 앞쪽 섹션은
그대로라 v1 리더가 읽던 부분은 바이트 위치까지 동일하다.
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
VERSION = 2          # v2: float32 기대 재구성/오차 섹션 추가
N_TESTVEC = 48


def c_array(name, values, ctype, per_line=12, fmt="%d"):
    body = []
    for i in range(0, len(values), per_line):
        body.append("    " + " ".join(fmt % v + "," for v in values[i:i + per_line]))
    return (f"static const {ctype} {name}[{len(values)}] = {{\n"
            + "\n".join(body) + "\n};\n")


def emit_header(qae, norm, thr_int, margin, path, ae=None, thr_f32=None):
    L = qae.layers
    n_layers = len(L)
    max_dim = max(LAYER_DIMS)
    w_bytes = sum(l.w_q.size for l in L)
    b_bytes = 4 * sum(l.b_q.size for l in L)
    norm_bytes = 4 * 2 * N_FEATURES
    # float32 레퍼런스 커널(system-design.md §5)용. 추론 로직은 int8만 쓰고,
    # 이쪽은 "같은 모델을 float로 돌리면 몇 사이클인가"를 재기 위해서만 존재한다.
    f32_bytes = 4 * sum(W.size + b.size for W, b in zip(ae.W, ae.b)) if ae else 0

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
      f" *   소계          {w_bytes + b_bytes + norm_bytes:6d} B   <- 추론에 실제로 쓰이는 것\n"
      f" *   float32 참조  {f32_bytes:6d} B   <- 커널 비교 전용(§5). 빼면 이만큼 준다\n"
      f" *   합계          {w_bytes + b_bytes + norm_bytes + f32_bytes:6d} B\n"
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

    if ae is not None:
        a("\n/* ══ float32 레퍼런스 커널 (system-design.md §5) ═══════════════════\n"
          " *\n"
          " * **추론 경로는 이걸 쓰지 않는다.** int8 순전파가 이 하드웨어에서 실제로\n"
          " * 이득인지 재기 위한 대조군일 뿐이다 — M4F는 FPU가 있어서 float MAC은\n"
          " * 명령 하나인 반면, 순수 C int8은 로드/부호확장/곱/누산에 재양자화\n"
          f" * (64비트 곱 + 라운딩 시프트)까지 붙는다. 그래서 {f32_bytes} B를 더 태워서\n"
          " * 같은 입력으로 두 경로를 재고, 어느 쪽이 나오든 그대로 기록한다.\n"
          " *\n"
          " * 양자화 전 원본 float64 가중치를 float32로 내린 값이다(학습 그대로).\n"
          " * 입력은 z-score, 출력도 z-score — 정수 경로처럼 격자를 맞출 필요가 없다.\n"
          " * 재구성 오차는 autoencoder.py recon_error()와 같은 **MSE**(24로 나눔)라\n"
          f" * 정수 경로의 AE_THRESHOLD_INT({int(thr_int)})와 단위가 다르다. 섞지 말 것. */\n")
        a(f"#define AE_THRESHOLD_F32    {float(thr_f32)!r}f\n\n")
        for li, (W, b) in enumerate(zip(ae.W, ae.b)):
            nout, nin = W.shape
            a(f"/* ── f32 layer {li}: {nin} → {nout}, act={LAYER_ACT[li]} ── */\n")
            a(c_array(f"ae_wf{li}", np.float32(W).reshape(-1), "float",
                      per_line=6, fmt="%.8ef"))
            a(c_array(f"ae_bf{li}", np.float32(b), "float", per_line=6, fmt="%.8ef"))
            a("\n")
        a("static const float *const ae_wf[AE_N_LAYERS] = { "
          + ", ".join(f"ae_wf{i}" for i in range(n_layers)) + " };\n")
        a("static const float *const ae_bf[AE_N_LAYERS] = { "
          + ", ".join(f"ae_bf{i}" for i in range(n_layers)) + " };\n")
        # 활성함수는 int8 쪽에선 act_min/act_max clamp에 흡수돼 있어서 별도 표가 없다.
        # float 쪽은 clamp가 없으니 relu 여부를 명시적으로 넘긴다.
        a(c_array("ae_is_relu", [1 if a_ == "relu" else 0 for a_ in LAYER_ACT],
                  "int32_t", per_line=8, fmt="%4d"))

    a("\n#endif /* MODEL_WEIGHTS_H */\n")

    path.write_text("".join(out))
    total = w_bytes + b_bytes + norm_bytes
    print(f"{path}  ({total} B ROM: 가중치 {w_bytes} + bias {b_bytes} + 정규화 {norm_bytes}"
          + (f", + float32 참조 {f32_bytes} B" if ae else "") + ")")
    return total


def emit_testvec(qae, norm, ae, windows, feats, path):
    norms = norm.transform(feats)
    q_in = qae.quantize_input(norms)
    q_out = qae.forward(q_in)
    err = qae.recon_error_int(q_in)
    # float32 대조군의 기대값. 입력은 정수 경로와 **같은** z-score라 두 커널이
    # 같은 것을 먹었을 때 뭘 내놓는지 나란히 비교된다.
    recon_f = ae.forward(norms)
    err_f = ((recon_f - norms) ** 2).mean(axis=1)
    n = len(feats)
    with open(path, "wb") as f:
        f.write(struct.pack("<6i", MAGIC, VERSION, n, WIN_LEN, N_CH, N_FEATURES))
        np.ascontiguousarray(windows, dtype="<f4").tofile(f)
        np.ascontiguousarray(feats, dtype="<f4").tofile(f)
        np.ascontiguousarray(norms, dtype="<f4").tofile(f)
        np.ascontiguousarray(q_in, dtype=np.int8).tofile(f)
        np.ascontiguousarray(q_out, dtype=np.int8).tofile(f)
        np.ascontiguousarray(err, dtype="<i4").tofile(f)
        np.ascontiguousarray(recon_f, dtype="<f4").tofile(f)
        np.ascontiguousarray(err_f, dtype="<f4").tofile(f)
    print(f"{path}  (v{VERSION}, {n}개 샘플, {path.stat().st_size} B)")


def emit_board_testvec(qae, norm, ae, windows, feats, labels, path):
    """보드 자가검증용 윈도우 2개를 헤더로 굽는다 (정상 1 + 이상 1).

    testvec.bin은 PC 전용이다 — 보드엔 파일시스템이 없다. 그렇다고 보드 추론을
    검증 없이 두면, 부팅해서 나온 숫자가 "맞는 답"인지 "그럴듯한 답"인지 구분할
    방법이 없다. 그래서 **기대 출력까지 같이 구운 윈도우 2개**를 넣어서, 부팅할 때
    보드가 스스로 PC와 대조하게 한다. 정상/이상을 하나씩 넣는 건 임계값 양쪽을
    다 밟아 판정 로직까지 확인하기 위해서다.

    Flash 비용: 2 × 128 × 6 × 4 B = 6,144 B. 512 KB 중 1.2%.
    """
    norms = norm.transform(feats)
    q_in = qae.quantize_input(norms)
    err_i = qae.recon_error_int(q_in)
    err_f = ((ae.forward(norms) - norms) ** 2).mean(axis=1)
    n = len(feats)

    out = []
    a = out.append
    a("/* 자동 생성 — `python3 export_header.py`. 손으로 고치지 말 것.\n"
      " *\n"
      " * 보드 추론 자가검증용 입력 윈도우 + PC가 계산한 기대 출력.\n"
      " * 쓰는 곳: firmware/Core/Src/ae_bench.c (부팅 시 1회, 스케줄러 전).\n"
      " * 커널 비교(kernel_compare.csv)의 입력도 이걸 쓴다 — 매번 같은 입력이라야\n"
      " * float 경로와 int8 경로가 같은 일을 한 것이 된다.\n"
      " *\n"
      f" * Flash: {n} × {WIN_LEN} × {N_CH} × 4 B = {n * WIN_LEN * N_CH * 4} B\n"
      " */\n")
    a("#ifndef AE_TESTVEC_BOARD_H\n#define AE_TESTVEC_BOARD_H\n\n"
      "#include <stdint.h>\n\n#include \"model_weights.h\"\n\n")
    a(f"#define AE_TV_N  {n}\n\n")
    a("/* 각 윈도우의 출처 라벨 (합성 proxy 데이터셋의 클래스명). */\n")
    a("static const char *const ae_tv_label[AE_TV_N] = { "
      + ", ".join(f'"{s}"' for s in labels) + " };\n\n")
    a("/* [샘플][WIN_LEN × N_CH] 채널 인터리브, 물리 단위(g / dps). */\n")
    a(f"static const float ae_tv_win[AE_TV_N][AE_WIN_LEN * AE_N_CH] = {{\n")
    for s in range(n):
        a("  {\n")
        flat = np.float32(windows[s]).reshape(-1)
        for i in range(0, len(flat), 6):
            a("    " + " ".join("%.8ef," % v for v in flat[i:i + 6]) + "\n")
        a("  },\n")
    a("};\n\n")
    a("/* PC 기대값. int8은 **비트정확**해야 하고(정수 연산이라 변명의 여지가 없다),\n"
      " * float은 Python float64 vs 보드 float32라 허용오차로 본다. */\n")
    a(c_array("ae_tv_err_int", [int(v) for v in err_i], "int32_t",
              per_line=4, fmt="%10d"))
    a(c_array("ae_tv_err_f32", np.float32(err_f), "float", per_line=4, fmt="%.8ef"))
    a("\n#endif /* AE_TESTVEC_BOARD_H */\n")

    path.write_text("".join(out))
    print(f"{path}  ({n}개 윈도우, {path.stat().st_size} B 소스)")


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

    # float 임계값은 학습 때 산정된 것을 그대로 쓴다(같은 검증셋 p99). 여기서 다시
    # 구하면 evaluate.py / accuracy.csv 의 ae_float32 행과 갈라진다.
    emit_header(qae, norm, int(round(thr_int)), margin, CREF_DIR / "model_weights.h",
                ae=ae, thr_f32=float(d["threshold"]))

    # 테스트벡터: 정상/이상을 섞어 임계값 양쪽을 다 밟게 한다
    test = load_split("test", with_windows=True)
    y = is_anomaly(test.labels)
    rng = np.random.default_rng(args.seed)
    pick = np.concatenate([
        rng.choice(np.flatnonzero(~y), N_TESTVEC // 2, replace=False),
        rng.choice(np.flatnonzero(y), N_TESTVEC - N_TESTVEC // 2, replace=False)])
    rng.shuffle(pick)
    emit_testvec(qae, norm, ae, test.windows[pick], test.feats[pick],
                 CREF_DIR / "testvec.bin")

    # 보드용은 2개만 — 정상/이상 각 1개. 판정이 확실히 갈리도록 오차가 임계값에서
    # 가장 먼 쪽을 고른다. 경계에 걸친 샘플을 고르면 float32 오차만으로 판정이
    # 뒤집혀서, 커널 버그와 반올림 차이를 구분할 수 없게 된다.
    err_all = qae.recon_error_int(qae.quantize_input(norm.transform(test.feats)))
    i_norm = int(np.flatnonzero(~y)[np.argmin(err_all[~y])])
    i_anom = int(np.flatnonzero(y)[np.argmax(err_all[y])])
    bpick = [i_norm, i_anom]
    emit_board_testvec(qae, norm, ae, test.windows[bpick], test.feats[bpick],
                       [str(v) for v in test.labels[bpick]],
                       CREF_DIR / "ae_testvec_board.h")


if __name__ == "__main__":
    main()
