#!/usr/bin/env python3
"""축 A 그래프 2장 → measurements/plots/*.png  (PLAN.md W4)

  inference_cycles_hist.png    추론 실행시간 / 판정지연 히스토그램 (config 대조)
  action_latency_by_config.png config별 액션 지연 — median/p99/max + 배치별 max

데이터 소스 (전부 이 리포에 있는 실측 원자료 — 보드 불필요):
  raw/w3_final_uart.log     [EDGE] 줄의 윈도우별 infer=/lat= (원시 샘플, 2026-08-08)
  raw/w3_sweep_batches.csv  config별 40배치 × (min/median/p99/max), 배치당 n=128 (2026-08-05)
  action_latency.csv        위 스윕의 config별 집계 (n=5120)

PLAN에는 "박스플롯"이라 적었지만 액션 지연의 원시 샘플은 보드 밖으로 안 나온다
(UART 대역폭 때문에 배치 통계만 내보냄 — measurement-methodology.md). 사분위수를
지어낼 수는 없으므로, 배치별 max 산점 + 집계 median/p99/max 마커로 같은 질문
("꼬리가 config에 따라 어떻게 움직이나")에 답한다. 값이 9µs~20ms로 3.5자릿수를
걸치므로 두 그림 다 로그 축 — 선형 축이면 B-mono의 20ms가 나머지를 전부 뭉갠다.

색은 model/plot.py와 같은 카테고리컬 팔레트를 공표된 슬롯 순서 그대로 쓴다(순환 금지).
같은 인접 페어리스트로 색각 이상 분리도 검증을 통과한 조합이다. 색은 통계량(엔티티)에
붙는다 — median/p99/max가 두 그림에서 같은 색을 유지한다.
"""

import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
RAW = HERE / "raw"
OUT = HERE / "plots"

CLOCK_HZ = 84_000_000
US = CLOCK_HZ / 1e6  # cycles per µs

# model/plot.py와 동일 팔레트·슬롯 순서. 통계량에 고정 배정.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
C_MED, C_P99, C_MAX = SERIES[0], SERIES[1], SERIES[2]
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#d8d7d2"
SURFACE = "#fcfcfb"

# 스윕 config 순서(고정): 대조군 → 정상 배치 → 역전 → 청킹 2단
CONFIGS = ["L0-noload", "A-mono", "B-mono", "C-chunk5ms", "D-chunk1ms"]
CONFIG_DESC = {
    "L0-noload": "부하 없음",
    "A-mono": "부하(37.7%)\n< 액션",
    "B-mono": "부하 > 액션\nmonolithic 20ms",
    "C-chunk5ms": "부하 > 액션\n5ms 청크×4",
    "D-chunk1ms": "부하 > 액션\n1ms 청크×20",
}


def setup_korean_font():
    """한글 라벨용 폰트 (model/plot.py와 동일 — WSL이면 Windows 폰트를 주워 쓴다)."""
    from matplotlib import font_manager as fm

    for cand in ("NanumGothic", "Noto Sans CJK KR", "Malgun Gothic", "AppleGothic"):
        if any(cand == f.name for f in fm.fontManager.ttflist):
            plt.rcParams["font.family"] = cand
            plt.rcParams["axes.unicode_minus"] = False
            return cand

    for path in ("/mnt/c/Windows/Fonts/malgun.ttf",
                 "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"):
        if Path(path).exists():
            fm.fontManager.addfont(path)
            name = fm.FontProperties(fname=path).get_name()
            plt.rcParams["font.family"] = name
            plt.rcParams["axes.unicode_minus"] = False
            print(f"  폰트: {name} ({path})")
            return name

    print("[경고] 한글 폰트가 없다 — 라벨이 □로 나온다.")
    return None


def _style(ax, title, xlabel, ylabel):
    ax.set_title(title, color=INK, fontsize=10.5, loc="left", pad=10)
    ax.set_xlabel(xlabel, color=INK2, fontsize=9)
    ax.set_ylabel(ylabel, color=INK2, fontsize=9)
    ax.tick_params(colors=INK2, labelsize=8, length=0)
    ax.grid(True, color=GRID, linewidth=0.8, alpha=0.7)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)


def _save(fig, path):
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)
    print(f"  {path.name}")


# ── 데이터 로드 ───────────────────────────────────────────────────────────────

EDGE_RE = re.compile(
    r"\[EDGE\]\*?\s+win=\d+\s+cfg=(\S+)\s+.*infer=([\d.]+)us\s+lat=([\d.]+)us")


def load_edge_samples():
    """w3_final_uart.log의 [EDGE] 줄 → {cfg: (infer_us[], lat_us[])}."""
    per = {}
    for line in (RAW / "w3_final_uart.log").read_text().splitlines():
        m = EDGE_RE.search(line)
        if m:
            cfg, infer, lat = m.group(1), float(m.group(2)), float(m.group(3))
            per.setdefault(cfg, ([], []))
            per[cfg][0].append(infer)
            per[cfg][1].append(lat)
    return {k: (np.array(v[0]), np.array(v[1])) for k, v in per.items()}


def load_sweep_batches():
    """w3_sweep_batches.csv의 action_latency 배치 → {cfg: max_us[]} (배치당 n=128)."""
    per = {c: [] for c in CONFIGS}
    for line in (RAW / "w3_sweep_batches.csv").read_text().splitlines():
        f = line.split(",")
        if f[0] == "action_latency" and len(f) >= 7 and f[1] in per:
            per[f[1]].append(int(f[6]) / US)   # max 열(사이클) → µs
    return {k: np.array(v) for k, v in per.items()}


def load_aggregate():
    """action_latency.csv의 2026-08-05 스윕 집계(n=5120/33024) → {cfg: dict(µs)}."""
    agg = {}
    for line in (HERE / "action_latency.csv").read_text().splitlines():
        if not line.startswith("action_latency,"):
            continue
        f = line.split(",")
        cfg = f[8]
        if cfg in CONFIGS and cfg not in agg:  # w3final 재측정(-w3final)은 제외
            agg[cfg] = {"n": int(f[1]), "median": int(f[3]) / US,
                        "p99": int(f[4]) / US, "max": int(f[5]) / US}
    return agg


# ── 그림 1: 추론 히스토그램 ───────────────────────────────────────────────────

def plot_inference_hist(edge):
    """추론 실행시간(infer)과 판정지연(lat) 히스토그램 — 스케줄링 조건별 그룹.

    config 5개를 낱개로 겹치면 안 읽히고, 낱개 꼬리를 단정하기엔 표본이 작다
    (같은 config라도 로그에 살아남은 샘플과 CSV 집계 배치가 서로 다른 시간대라,
    A-mono 꼬리가 CSV엔 있고 여기엔 없다). 표본이 커도 성립하는 구분만 쓴다 —
    부하 태스크가 추론(prio 12)보다 **아래**인가 **위**인가, 위라면 청크가 얼마나
    잘게 쪼개져 있는가. 읽을 것: min은 모든 그룹에서 713µs(커널은 상수,
    kernel_compare.csv에서 입력 의존성 0.6% 이내로 확인). 부하가 위로 가면
    실행 자체가 선점당해 꼬리가 25ms까지 가고, 1ms 청킹은 median까지 민다.
    """
    groups = [
        ("부하 < 추론 (L0·A-mono)", ["L0-noload", "A-mono"], SERIES[0]),
        ("부하 > 추론, 통짜·5ms (B·C)", ["B-mono", "C-chunk5ms"], SERIES[1]),
        ("부하 > 추론, 1ms 청킹 (D)", ["D-chunk1ms"], SERIES[2]),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 4.1), dpi=150)
    fig.patch.set_facecolor(SURFACE)

    titles = [("추론 실행시간 (윈도우당)", 0), ("윈도우 완성 → 판정 확정 지연", 1)]
    for ax, (title, idx) in zip(axes, titles):
        ax.set_facecolor(SURFACE)
        allv = np.concatenate([edge[c][idx] for _, cfgs, _ in groups for c in cfgs])
        bins = np.logspace(np.log10(allv.min() * 0.97),
                           np.log10(allv.max() * 1.08), 40)
        # 세 그룹이 713µs 빈을 공유해 겹치면 아래 그룹이 가려진다 → 스택.
        # 세그먼트 경계는 서페이스색 1px 선으로 분리(겹침 마크의 서페이스 링 규칙).
        vals = [np.concatenate([edge[c][idx] for c in cfgs]) for _, cfgs, _ in groups]
        ax.hist(vals, bins=bins, stacked=True,
                color=[c for _, _, c in groups],
                edgecolor=SURFACE, linewidth=0.6,
                label=[f"{lbl}  n={len(v)}" for (lbl, _, _), v in zip(groups, vals)])
        ax.set_xscale("log")
        ax.set_yscale("symlog", linthresh=10)   # 꼬리(1~3개 빈)가 안 보이면 그림의 목적이 사라진다
        ax.set_ylim(0, None)
        _style(ax, title, "µs [log]", "윈도우 수 [symlog]" if idx == 0 else "")
        ax.legend(frameon=False, fontsize=8, labelcolor=INK2)

    axes[0].annotate("min은 전 config 713 µs\n— 커널 자체는 상수",
                     xy=(730, 60), xytext=(14, -40), textcoords="offset points",
                     ha="left", color=INK2, fontsize=8,
                     arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    d_med = float(np.median(edge["D-chunk1ms"][0]))
    axes[0].annotate(f"1ms 청킹은 median까지 민다\n(713 → {d_med:.0f} µs)",
                     xy=(d_med, 20), xytext=(18, -30), textcoords="offset points",
                     ha="left", color=INK2, fontsize=8,
                     arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    bc_lat = np.concatenate([edge["B-mono"][1], edge["C-chunk5ms"][1]])
    axes[1].annotate(f"기상까지 밀린 꼬리 max {bc_lat.max()/1000:.1f} ms",
                     xy=(bc_lat.max(), 1), xytext=(-6, 30),
                     textcoords="offset points", ha="right",
                     color=INK2, fontsize=8,
                     arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))

    fig.suptitle("int8 추론 — 커널은 상수, 꼬리는 스케줄링이 만든다 "
                 "(w3_final_uart.log 윈도우별 원시 샘플, 공식 집계는 inference_cycles.csv)",
                 color=INK, fontsize=10.5, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    _save(fig, OUT / "inference_cycles_hist.png")


# ── 그림 2: config별 액션 지연 ────────────────────────────────────────────────

def plot_action_latency(batches, agg):
    """x=config, y=지연(µs, log). 집계 median/p99/max 마커 + 배치별 max 산점.

    부하율은 전 config 동일(37.7%, L0 제외) — 남는 차이는 우선순위 배치와
    선점 지점뿐이다. 읽을 것: median은 어디서나 ~9.2µs로 같고(D만 14.4µs),
    max만 우선순위 역전(B)에서 1,166배가 된다. 청킹(C·D)은 그 꼬리를
    1/4·1/20로 자르되 공짜가 아니다(D는 median까지 오른다).
    """
    fig, ax = plt.subplots(figsize=(8.8, 5.0), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    x = np.arange(len(CONFIGS))
    rng = np.random.default_rng(0)  # 지터 재현성

    for i, cfg in enumerate(CONFIGS):
        bmax = batches[cfg]
        jitter = rng.uniform(-0.13, 0.13, len(bmax))
        ax.scatter(np.full(len(bmax), i) + jitter, bmax, s=12, color=INK2,
                   alpha=0.35, linewidths=0, zorder=2,
                   label="배치별 max (n=128/배치)" if i == 0 else None)

    for key, color, marker, ms in (("median", C_MED, "o", 7),
                                   ("p99", C_P99, "D", 6),
                                   ("max", C_MAX, "^", 8)):
        vals = [agg[c][key] for c in CONFIGS]
        ax.plot(x, vals, color=color, linewidth=1.4, alpha=0.55, zorder=3)
        ax.scatter(x, vals, s=ms**2, color=color, marker=marker, zorder=4,
                   edgecolors=SURFACE, linewidths=1.2,
                   label=f"{key} (config당 n={agg['A-mono']['n']:,})")

    ax.axhline(1000, color=INK, linewidth=1.2, linestyle="--", zorder=1)
    ax.text(-0.42, 1080, "관측 기준선 1 ms", ha="left", va="bottom",
            color=INK, fontsize=8.5)

    # 핵심 수치 직접 라벨 (선택적 — 전 점 라벨링 금지)
    b = agg["B-mono"]["max"]
    ax.annotate(f"max {b/1000:.1f} ms\n= 무부하 대비 1,166배",
                xy=(2, b), xytext=(14, -4), textcoords="offset points",
                color=INK, fontsize=8.5)
    d = agg["D-chunk1ms"]
    ax.annotate(f"max {d['max']:.0f} µs (B의 1/20)\nmedian은 {d['median']:.1f} µs로 악화",
                xy=(4, d["max"]), xytext=(-14, 26), textcoords="offset points",
                ha="right", color=INK2, fontsize=8)

    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels([f"{c}\n{CONFIG_DESC[c]}" for c in CONFIGS],
                       fontsize=8, color=INK2)
    _style(ax, "마감시한 이벤트 → 로컬 액션 지연 — 부하량이 아니라 우선순위 배치가 꼬리를 만든다",
           "", "지연 µs [log]")
    ax.legend(frameon=False, fontsize=8.5, loc="upper left", labelcolor=INK2)
    fig.tight_layout()
    _save(fig, OUT / "action_latency_by_config.png")


def main():
    setup_korean_font()
    OUT.mkdir(parents=True, exist_ok=True)
    print(f"그래프 → {OUT}")

    edge = load_edge_samples()
    batches = load_sweep_batches()
    agg = load_aggregate()

    # 로드 자가검증: 집계 CSV와 배치 원자료가 서로 맞는지 (단발 신뢰 금지의 로드 판)
    for cfg in CONFIGS:
        assert len(batches[cfg]) == 40, f"{cfg}: 배치 수 {len(batches[cfg])} != 40"
        assert abs(batches[cfg].max() - agg[cfg]["max"]) < 0.5, \
            f"{cfg}: 배치 max({batches[cfg].max():.1f}) != 집계 max({agg[cfg]['max']:.1f})"

    plot_inference_hist(edge)
    plot_action_latency(batches, agg)


if __name__ == "__main__":
    main()
