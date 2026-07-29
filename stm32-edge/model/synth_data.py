#!/usr/bin/env python3
"""합성 IMU 스트림 생성기 — **보드 실측 데이터의 임시 대역(stand-in)**.

⚠️ 이 파일이 만드는 데이터는 실제 MPU6050 캡처가 아니다. W2 브링업(I2C 100Hz 스트리밍)이
   끝나기 전에 학습·양자화·export·C검증 파이프라인을 먼저 세우기 위한 proxy다.
   PLAN.md §5 리스크표의 "추론 입력을 합성 버퍼로 대체" 항목이 바로 이 경로.

**실측으로 갈아끼우는 법**: 보드 UART로 아래 CSV를 세션마다 하나씩 뽑아 data/ 에 넣고
manifest.csv에 (파일, split, label) 한 줄씩 추가하면 끝. 이 스크립트를 다시 돌릴 필요 없다.

    CSV 포맷 (헤더 1줄 + '#' 주석 허용):
        t_ms,ax,ay,az,gx,gy,gz
        가속도 = g, 자이로 = dps (물리 단위로 이미 변환된 값)

**왜 한 조건당 파일 여러 개(세션)인가** — 이게 이 스크립트 설계의 핵심이다.
장착 기울기, 자이로 바이어스, 회전수 같은 값은 **한 번 붙여놓고 녹화하는 동안은 거의 고정**이고,
센서를 다시 붙이면 바뀐다. 한 조건을 긴 파일 하나로 뽑으면 이 런-단위 상수의 산포가
학습셋에 안 들어가서
  (1) 멀쩡한 정상 세션이 이상으로 뜨고,
  (2) 모델이 "진동 성격"이 아니라 "그날의 장착 기울기"로 이상을 맞히는 누수가 생긴다.
실제로 첫 버전(조건당 파일 1개)에서 정상 테스트 |z|max가 26까지 튀고, imbalance가
az.mean(=기울기) 하나로 분리되는 걸 확인했다. 그래서 조건당 짧은 세션 여러 개로 바꿨다.

모델링한 대상: 소형 모터/선풍기의 회전 진동.
  - 회전 기본 주파수 f0 ≈ 12 Hz (약 720 RPM) + 2x·3x 고조파
  - 100 Hz 샘플링 → 나이퀴스트 50 Hz. 3x = 36 Hz 까지만 쓰므로 에일리어싱 없음
    (timeseries-windowing.md §2). 실측 f0가 16 Hz를 넘으면 3x가 접히니 브링업 때 확인할 것.
  - 반경 방향(ax, ay)에 진동이 크고, 축 방향(az)엔 중력 1g가 상시 실린다

이상 4종은 전부 "손으로 흉내낼 수 있는" 것들로 골랐다(발표 시 재현 가능):
  imbalance  회전체 불균형     → 1x 성분만 크게 증가 (테이프/클립 한 쪽에 붙이기)
  bearing    베어링 마모       → 광대역 고주파 + 공진대 에너지 증가
  impact     충격/헐거움       → 산발적 감쇠 임펄스 (툭툭 치기)
  looseness  마운트 헐거움     → 저주파 wander + p2p 증가 (받침 흔들기)
"""

import argparse

import numpy as np

from config import DATA_DIR, FS_HZ, SEED

F0_HZ = 12.0          # 회전 기본 주파수
GRAVITY_G = 1.0       # az에 상시 실리는 중력

# (조건 이름, split, 라벨, 세션 수, 세션당 초)
CONDITIONS = (
    ("normal_train", "train", "normal", 10, 45.0),
    ("normal_test", "test", "normal", 5, 45.0),
    ("anom_imbalance", "test", "imbalance", 3, 30.0),
    ("anom_bearing", "test", "bearing", 3, 30.0),
    ("anom_impact", "test", "impact", 3, 30.0),
    ("anom_looseness", "test", "looseness", 3, 30.0),
)


def _slow_walk(n, rng, sigma, tau_s):
    """평균 0 주위를 천천히 도는 1차 저역 랜덤워크 — 런 내 조건 변동(온도·부하) 모사."""
    a = np.exp(-1.0 / (tau_s * FS_HZ))
    noise = rng.normal(0.0, sigma * np.sqrt(1 - a * a), n)
    out = np.empty(n)
    acc = 0.0
    for i in range(n):
        acc = a * acc + noise[i]
        out[i] = acc
    return out


def _impulses(n, rng, rate_hz, amp, ring_hz, tau_s):
    """감쇠 진동 임펄스 열 — 충격/헐거움."""
    out = np.zeros(n)
    n_hits = rng.poisson(rate_hz * n / FS_HZ)
    ring_len = int(6 * tau_s * FS_HZ)
    t = np.arange(ring_len) / FS_HZ
    env = np.exp(-t / tau_s) * np.sin(2 * np.pi * ring_hz * t)
    for _ in range(n_hits):
        k = int(rng.integers(0, n))
        a = amp * rng.uniform(0.6, 1.6)
        end = min(n, k + ring_len)
        out[k:end] += a * env[: end - k]
    return out


def generate_session(duration_s, kind, rng):
    """한 번의 '녹화 세션'(센서를 붙였다 뗄 때까지)을 만든다. 반환 data[n, 6]."""
    n = int(duration_s * FS_HZ)
    t = np.arange(n) / FS_HZ

    # ── 런 단위 nuisance 상수: 세션마다 새로 뽑힌다 ──────────────────────
    # 이상 여부와 무관해야 한다. 정상 학습셋에도 같은 산포로 들어가므로
    # 모델이 이걸로 이상을 맞힐 수 없다.
    tilt = rng.uniform(-0.06, 0.06)              # 장착 기울기 → az 평균 변화
    gz_bias = rng.normal(0.0, 0.9)               # 자이로 z 바이어스 (dps)
    gxy_bias = rng.normal(0.0, 0.4, 2)
    f0 = F0_HZ * rng.uniform(0.93, 1.07)         # 회전수 개체차

    phase = rng.uniform(0, 2 * np.pi, 3)
    f_inst = f0 * (1.0 + 0.02 * _slow_walk(n, rng, 1.0, 8.0))   # 회전수 미세 흔들림
    theta = 2 * np.pi * np.cumsum(f_inst) / FS_HZ
    gain = 1.0 + 0.10 * _slow_walk(n, rng, 1.0, 20.0)           # 부하 변동

    # 정상 진폭 (g / dps)
    a1, a2, a3 = 0.020, 0.008, 0.004
    hf_noise = 0.0035
    g1 = 0.60

    # ── 이상 종류별 변조 ────────────────────────────────────────────────
    imb = 1.0                 # 1x 성분 배율
    hf_gain = 1.0             # 광대역 고주파 배율
    reso = 0.0                # 공진대(38 Hz) 진폭
    wander = np.zeros(n)      # 저주파 wander
    shock = np.zeros(n)       # 임펄스

    if kind == "imbalance":
        imb = rng.uniform(2.2, 3.4)
    elif kind == "bearing":
        hf_gain = rng.uniform(2.5, 4.5)
        reso = rng.uniform(0.005, 0.010)
    elif kind == "impact":
        shock = _impulses(n, rng, rate_hz=1.5, amp=0.07, ring_hz=40.0, tau_s=0.025)
    elif kind == "looseness":
        wander = rng.uniform(0.015, 0.028) * _slow_walk(n, rng, 1.0, 0.35)
        imb = rng.uniform(1.25, 1.6)
    elif kind != "normal":
        raise ValueError(f"unknown anomaly kind: {kind}")

    reso_sig = (reso * np.sin(2 * np.pi * 38.0 * t + rng.uniform(0, 2 * np.pi))
                if reso else 0.0)

    def radial(ph_off, sign):
        return (gain * (imb * a1 * np.sin(theta + phase[0] + ph_off)
                        + a2 * np.sin(2 * theta + phase[1])
                        + a3 * np.sin(3 * theta + phase[2]))
                + hf_gain * rng.normal(0, hf_noise, n)
                + reso_sig + sign * wander + sign * shock)

    ax = radial(0.0, +1.0)
    ay = radial(np.pi / 2, -1.0)
    az = (GRAVITY_G * np.cos(tilt)
          + 0.35 * gain * a1 * np.sin(theta + phase[0])
          + 0.5 * hf_gain * rng.normal(0, hf_noise, n)
          + 0.3 * shock)
    # 자이로: 회전 진동이 각속도로도 보인다. gz(축 회전)는 거의 정지
    gx = (gxy_bias[0] + gain * g1 * imb * np.sin(theta + phase[0] + 0.3)
          + hf_gain * rng.normal(0, 0.12, n) + 40 * shock)
    gy = (gxy_bias[1] + gain * g1 * np.sin(theta + phase[1] - 0.4)
          + hf_gain * rng.normal(0, 0.12, n) - 40 * shock)
    gz = (gz_bias + rng.normal(0, 0.06, n)
          + 0.8 * _slow_walk(n, rng, 1.0, 30.0) + 12 * shock)

    return np.column_stack([ax, ay, az, gx, gy, gz])


def write_csv(path, data, label):
    t_ms = np.arange(len(data)) * (1000.0 / FS_HZ)
    with open(path, "w") as f:
        f.write("# SYNTHETIC PROXY DATA — 실제 MPU6050 캡처가 아님 (synth_data.py 생성)\n")
        f.write(f"# label={label} fs_hz={FS_HZ} f0_hz~{F0_HZ} units=g,dps\n")
        f.write("t_ms,ax,ay,az,gx,gy,gz\n")
        for i in range(len(data)):
            f.write("%.1f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f\n" % (t_ms[i], *data[i]))


def main():
    ap = argparse.ArgumentParser(description="합성 IMU 세션 생성 (보드 실측 대역)")
    ap.add_argument("--seed", type=int, default=SEED)
    args = ap.parse_args()

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    manifest = ["file,split,label"]

    for cond, split, label, n_sess, dur in CONDITIONS:
        for s in range(n_sess):
            name = f"{cond}_{s:02d}.csv"
            write_csv(DATA_DIR / name, generate_session(dur, label, rng), label)
            manifest.append(f"{name},{split},{label}")
        print(f"{cond:16s} split={split:5s} label={label:10s} "
              f"{n_sess}세션 × {dur:.0f}s")

    (DATA_DIR / "manifest.csv").write_text("\n".join(manifest) + "\n")
    print(f"\nmanifest.csv: {len(manifest) - 1}개 세션")
    print("실측 데이터를 넣을 땐 CSV를 data/에 두고 manifest.csv에 한 줄 추가하면 된다.")


if __name__ == "__main__":
    main()
