#!/usr/bin/env python3
"""보드 수집 모드의 UART 로그 → data/*.csv + manifest.csv

    python3 import_board_csv.py ~/w3_collect.log

**왜 이 스크립트가 필요한가**: W3 실측에서 합성 데이터로 학습한 모델이 실제 센서를
전혀 설명하지 못한다는 게 드러났다 — 장착 기울기 2.3도가 학습 분포 기준 106σ,
합성 생성기가 가정한 센서 노이즈가 실제의 7.3배(docs/system-design.md §6).
그래서 실측으로 갈아끼운다. 이 파일 아래의 파이프라인(학습·양자화·export·검증)은
**한 줄도 안 바뀐다** — dataset.py 위로는 합성이든 실측이든 구분이 없게 짜뒀다.

펌웨어가 raw LSB로 뱉는 이유와 여기서 환산하는 이유:
  - 펌웨어는 `--specs=nano.specs` 빌드라 printf("%f")가 링크에서 빠져 있다.
  - 정수가 짧아 UART 대역폭도 아낀다(한 줄 ~4.3ms < 10ms 예산).
  - 환산 상수를 **로그 헤더에서 읽어 쓴다.** 여기에 손으로 16384를 박아두면
    나중에 펌웨어에서 AFS_SEL을 올렸을 때 조용히 4배 틀린 데이터가 만들어진다.
"""

import argparse
import re
import sys
from collections import Counter

import numpy as np

from config import CHANNELS, DATA_DIR, FS_HZ, HOP, N_CH, WIN_LEN

BEGIN = re.compile(r"^#SESSION_BEGIN,(\d+),(\w+),(\d+),(\d+)")
END = re.compile(r"^#SESSION_END,(\d+),(\w+),(\d+),(\d+),(\d+),(\d+)(?:,(\d+))?")
CFG = re.compile(r"AFS_SEL=(\d+), FS_SEL=(\d+), DLPF_CFG=(\d+)")


def parse_log(path):
    """로그 → (설정, [세션...]). 세션 = dict(idx,label,fs,rows,t,fail,sat,late,recov).

    같은 로그에 부팅이 여러 번 들어 있을 수 있다(리셋하고 다시 시작한 경우).
    그래서 세션을 idx로 식별하지 않고 **등장 순서대로** 모은다 — idx로 묶으면
    두 번째 부팅의 세션 0이 첫 번째 것을 덮어써 데이터가 조용히 사라진다.
    """
    cfg, sessions, cur = None, [], None
    for raw in open(path, errors="replace"):
        line = raw.strip()
        if cfg is None:
            m = CFG.search(line)
            if m:
                cfg = tuple(int(v) for v in m.groups())
                continue
        m = BEGIN.match(line)
        if m:
            cur = dict(idx=int(m.group(1)), label=m.group(2), fs=int(m.group(3)),
                       want=int(m.group(4)), rows=[], t=[],
                       fail=0, sat=0, late=0, recov=0)
            continue
        m = END.match(line)
        if m and cur is not None:
            cur.update(fail=int(m.group(4)), sat=int(m.group(5)), late=int(m.group(6)),
                       recov=int(m.group(7) or 0))
            sessions.append(cur)
            cur = None
            continue
        if cur is not None and line and line[0].isdigit():
            p = line.split(",")
            if len(p) == 1 + N_CH:
                try:
                    cur["t"].append(int(p[0]))
                    cur["rows"].append([int(v) for v in p[1:]])
                except ValueError:
                    cur["t"].pop() if len(cur["t"]) > len(cur["rows"]) else None
    if cur is not None:
        print(f"  [경고] 세션 {cur['idx']}({cur['label']})가 #SESSION_END 없이 끝났다 "
              f"— 로그가 잘렸거나 보드가 멈춘 것. 버린다.", file=sys.stderr)
    return cfg, sessions


def split_contiguous(t, rows, fs_hz):
    """t_ms가 끊긴 곳에서 자른다. 반환 [(rows_run, n_samples), ...].

    **왜 반드시 잘라야 하나**: I2C 읽기가 실패하면 그 샘플은 기록되지 않는다(앞 값을
    복제하면 std/mad가 거짓이 되므로 일부러 그렇게 짰다). 그래서 실패 구간을 사이에 둔
    두 조각이 파일에서는 붙어 있는데 **실제 시간축에서는 떨어져 있다.** 그 경계를 걸친
    윈도우는 존재하지 않는 급변을 하나 만들어내고, 하필 mad(인접 차분)가 그걸 그대로
    특징으로 삼는다 — 즉 배선 사고가 '이상 신호'로 학습된다.
    dataset.py가 세션 경계를 넘지 않는 것과 같은 이유이고, 여기서는 더 잘게 나눌 뿐이다.
    """
    step = 1000 // fs_hz
    runs, start = [], 0
    for k in range(1, len(t)):
        if t[k] - t[k - 1] != step:
            runs.append(rows[start:k])
            start = k
    runs.append(rows[start:])
    return [r for r in runs if len(r) >= WIN_LEN]


def main():
    ap = argparse.ArgumentParser(description="보드 UART 수집 로그 → 학습 데이터셋")
    ap.add_argument("log", help="수집 모드 UART 로그 파일")
    ap.add_argument("--prefix", default="board", help="생성할 CSV 파일 접두어")
    ap.add_argument("--dry-run", action="store_true", help="쓰지 않고 요약만")
    ap.add_argument("--outlier-x", type=float, default=5.0,
                    help="정상 세션 오염 판정 배수 (기본 5배)")
    ap.add_argument("--keep-outliers", action="store_true",
                    help="오염 의심 normal 세션도 그대로 쓴다 (권장하지 않음)")
    args = ap.parse_args()

    cfg, sessions = parse_log(args.log)
    if not sessions:
        sys.exit(f"{args.log}: #SESSION_BEGIN 블록을 못 찾았다. 수집 모드 로그가 맞나?")
    if cfg is None:
        sys.exit("로그에서 AFS_SEL/FS_SEL을 못 읽었다. 환산 상수를 확정할 수 없으니 중단한다 "
                 "— 여기서 기본값을 가정하면 조용히 틀린 데이터가 만들어진다.")

    afs, fs_sel, dlpf = cfg
    acc_lsb_per_g = 16384 >> afs
    gyr_lsb_per_dps = 131.0 / (1 << fs_sel)
    print(f"센서 설정(로그에서 읽음): AFS_SEL={afs} (±{2 << afs}g, {acc_lsb_per_g} LSB/g), "
          f"FS_SEL={fs_sel} (±{250 << fs_sel}dps, {gyr_lsb_per_dps:g} LSB/dps), DLPF_CFG={dlpf}")

    # split 배정: normal은 마지막 하나만 test로 뺀다.
    #   - 학습(AE)은 정상만 쓴다(비지도) -> 정상 표본을 최대한 train에
    #   - 그래도 test에 정상이 하나는 있어야 FPR을 계산할 수 있다
    normal_idx = [i for i, s in enumerate(sessions) if s["label"] == "normal"]
    if len(normal_idx) < 2:
        sys.exit("normal 세션이 2개 미만이다. 학습용과 평가용을 나눌 수 없다.")

    # ── 라벨 오염 검사 ────────────────────────────────────────────────────────
    # "normal"이라고 적혀 있는데 실제로는 움직인 세션을 잡는다. 수집 중 사람이
    # 보드를 만졌거나, 안내를 놓쳤거나, 리셋 직후 손에 들고 있었거나 —
    # 실측(2026-08-08)에서 실제로 첫 세션이 다른 정상의 86배로 흔들려 있었다.
    #
    # **이게 왜 치명적인가**: 오토인코더는 정상만 보고 "정상의 모양"을 배운다.
    # 흔들린 구간이 학습셋에 들어가면 모델은 흔들림을 정상으로 배우고, 그러면
    # 이상 탐지가 조용히 무력해진다 — 정확도 지표는 그럴듯하게 나오면서.
    # 그래서 라벨을 믿지 않고 **데이터에게 라벨이 맞는지 물어본다.**
    def motion(s):
        a = np.asarray(s["rows"], dtype=np.float64)[:, 0:3] / acc_lsb_per_g
        return float(a.std(axis=0).mean())

    mov = {i: motion(sessions[i]) for i in normal_idx}
    med = float(np.median(list(mov.values())))
    bad = {i for i in normal_idx if mov[i] > args.outlier_x * med}
    if bad:
        print(f"\n[오염 의심] normal 세션의 움직임 중앙값 {med:.5f} g 대비 "
              f"{args.outlier_x:g}배 초과:")
        for i in sorted(bad):
            print(f"  세션 {i}: {mov[i]:.5f} g ({mov[i] / med:.0f}배) "
                  f"— '{'그대로 쓴다 (--keep-outliers)' if args.keep_outliers else '제외한다'}'")
        if not args.keep_outliers:
            print("  이유: 흔들린 구간이 학습셋에 들어가면 오토인코더가 흔들림을 "
                  "정상으로 배운다.")
    if args.keep_outliers:
        bad = set()

    good_normal = [i for i in normal_idx if i not in bad]
    if len(good_normal) < 2:
        sys.exit("오염을 빼고 나니 normal 세션이 2개 미만이다. 다시 수집할 것.")
    test_normal = good_normal[-1]

    rows_out, total_win = [], Counter()
    print(f"\n{'#':>3} {'라벨':<11} {'split':<6} {'샘플':>6} {'조각':>4} {'윈도우':>6} "
          f"{'fail':>5} {'sat':>4} {'recov':>5}")
    for i, s in enumerate(sessions):
        n = len(s["rows"])
        if i in bad:
            print(f"{i:>3} {s['label']:<11} {'제외':<6} {n:>6}    -      - "
                  f"      -    -     -  <- 오염 의심")
            continue
        split = "test" if (s["label"] != "normal" or i == test_normal) else "train"
        runs = split_contiguous(s["t"], s["rows"], s["fs"])
        nwin = sum((len(r) - WIN_LEN) // HOP + 1 for r in runs)
        flag = ""
        if s["fail"]:
            flag += f"  <- I2C 실패 {s['fail']}"
        if s["sat"]:
            flag += f"  <- 포화 {s['sat']}샘플(±{2 << afs}g 한계 접촉)"
        if s["recov"]:
            flag += f"  <- 버스 복구 {s['recov']}회"
        if len(runs) != 1:
            flag += f"  <- **{len(runs)}조각으로 잘림**"
        print(f"{i:>3} {s['label']:<11} {split:<6} {n:>6} {len(runs):>4} {nwin:>6} "
              f"{s['fail']:>5} {s['sat']:>4} {s['recov']:>5}{flag}")
        total_win[(split, s["label"])] += nwin

        # 조각마다 별도 파일 = dataset.py 관점에서 별도 세션. 윈도우가 조각 경계를
        # 넘지 않는 게 보장된다.
        for j, r in enumerate(runs):
            suffix = "" if len(runs) == 1 else chr(ord("a") + j)
            fname = f"{args.prefix}_{i:02d}{suffix}_{s['label']}.csv"
            rows_out.append((fname, split, s["label"], np.asarray(r, dtype=np.int64)))

    print("\n윈도우 합계:")
    for k in sorted(total_win):
        print(f"  {k[0]:<6} {k[1]:<11} {total_win[k]:>5}")
    n_tr = sum(v for k, v in total_win.items() if k[0] == "train")
    print(f"\n학습용 정상 윈도우 {n_tr}개 -> 검증(임계값 산정) 25% = 약 {int(n_tr * .25)}개.")
    if n_tr * 0.25 < 50:
        print("  [경고] 검증 표본이 50개 미만이면 p99 임계값이 사실상 최댓값이 된다 "
              "— 오경보는 줄지만 재현율이 낮게 나온다. 정상 세션을 더 수집할 것.")

    if args.dry_run:
        print("\n--dry-run: 아무것도 쓰지 않았다.")
        return

    # 합성 manifest를 덮기 전에 보존한다. 합성/실측을 섞으면 분포가 다른 두 세계가
    # 한 학습셋에 들어가고, 그건 우리가 지금 고치려는 문제를 더 키우는 짓이다.
    man = DATA_DIR / "manifest.csv"
    synth = DATA_DIR / "manifest_synth.csv"
    if man.exists() and not synth.exists():
        synth.write_text(man.read_text())
        print(f"\n기존(합성) manifest를 {synth.name}로 보존했다.")

    hdr = "t_ms," + ",".join(CHANNELS)
    for fname, _split, _label, arr in rows_out:
        phys = arr.astype(np.float64)
        phys[:, 0:3] /= acc_lsb_per_g      # ax,ay,az -> g
        phys[:, 3:6] /= gyr_lsb_per_dps    # gx,gy,gz -> dps
        t = (np.arange(len(arr)) * 1000 // FS_HZ).reshape(-1, 1)
        out = np.hstack([t, phys])
        np.savetxt(DATA_DIR / fname, out, delimiter=",", header=hdr, comments="",
                   fmt=["%d"] + ["%.6f"] * N_CH)

    man.write_text("file,split,label\n" +
                   "".join(f"{f},{s},{l}\n" for f, s, l, _ in rows_out))
    print(f"{man} 갱신: 세션 {len(rows_out)}개")

    # 즉시 눈으로 검증할 수 있는 요약. 여기서 정상/이상의 std가 안 갈리면
    # 학습을 돌려봐야 시간만 버린다.
    print("\n=== 수집 데이터 요약 (물리 단위) ===")
    print(f"{'라벨':<11} {'ax.mean':>9} {'ay.mean':>9} {'az.mean':>9} "
          f"{'ax.std':>9} {'gx.mean':>9}")
    for fname, _s, label, arr in rows_out:
        p = arr.astype(np.float64)
        p[:, 0:3] /= acc_lsb_per_g
        p[:, 3:6] /= gyr_lsb_per_dps
        print(f"{label:<11} {p[:,0].mean():9.5f} {p[:,1].mean():9.5f} {p[:,2].mean():9.5f} "
              f"{p[:,0].std():9.5f} {p[:,3].mean():9.4f}")
    print("\n정상 대비 이상의 ax.std가 확실히 커야 한다. 안 그러면 흔들기가 약했던 것이다.")
    print("\n다음: SKIP_SYNTH=1 ./run_all.sh")


if __name__ == "__main__":
    main()
