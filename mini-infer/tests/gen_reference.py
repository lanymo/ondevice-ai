#!/usr/bin/env python3
"""NumPy 레퍼런스 테스트 벡터 생성기.

tests/data/ 에 바이너리 케이스 파일을 만든다. test_correctness가 읽어서
C++ 나이브 커널 출력과 비교한다.

파일 포맷 (리틀엔디언):
  matmul_*.bin : int32 M,K,N  다음 float32 A[M*K], B[K*N], C_expected[M*N]
  conv2d_*.bin : int32 cin,h,w,cout,kh,kw,stride,pad
                 다음 float32 in, weight, bias, out_expected
레퍼런스는 float64로 누적 후 float32로 캐스팅 (누적 순서 차이 흡수).
"""

from pathlib import Path

import numpy as np

DATA_DIR = Path(__file__).resolve().parent / "data"
rng = np.random.default_rng(42)


def write_case(path: Path, ints, arrays):
    with open(path, "wb") as f:
        np.asarray(ints, dtype="<i4").tofile(f)
        for a in arrays:
            np.ascontiguousarray(a, dtype="<f4").tofile(f)


def conv2d_ref(x, w, b, stride, pad):
    cin, h, wd = x.shape
    cout, _, kh, kw = w.shape
    xp = np.pad(x.astype(np.float64), ((0, 0), (pad, pad), (pad, pad)))
    oh = (h + 2 * pad - kh) // stride + 1
    ow = (wd + 2 * pad - kw) // stride + 1
    out = np.empty((cout, oh, ow), dtype=np.float64)
    w64 = w.astype(np.float64)
    for oy in range(oh):
        for ox in range(ow):
            region = xp[:, oy * stride:oy * stride + kh, ox * stride:ox * stride + kw]
            out[:, oy, ox] = np.tensordot(w64, region, axes=([1, 2, 3], [0, 1, 2]))
    return (out + b.astype(np.float64)[:, None, None]).astype(np.float32)


def gen_matmul(name, m, k, n):
    a = rng.uniform(-1, 1, (m, k)).astype(np.float32)
    b = rng.uniform(-1, 1, (k, n)).astype(np.float32)
    c = (a.astype(np.float64) @ b.astype(np.float64)).astype(np.float32)
    write_case(DATA_DIR / f"matmul_{name}.bin", [m, k, n], [a, b, c])
    print(f"matmul_{name}: M={m} K={k} N={n}")


def gen_conv2d(name, cin, h, w, cout, kh, kw, stride, pad):
    x = rng.uniform(-1, 1, (cin, h, w)).astype(np.float32)
    wgt = rng.uniform(-1, 1, (cout, cin, kh, kw)).astype(np.float32)
    b = rng.uniform(-1, 1, cout).astype(np.float32)
    out = conv2d_ref(x, wgt, b, stride, pad)
    write_case(DATA_DIR / f"conv2d_{name}.bin",
               [cin, h, w, cout, kh, kw, stride, pad], [x, wgt, b, out])
    print(f"conv2d_{name}: cin={cin} {h}x{w} -> cout={cout} "
          f"k={kh}x{kw} s={stride} p={pad} out={out.shape}")


def main():
    DATA_DIR.mkdir(exist_ok=True)

    gen_matmul("small_odd", 4, 5, 6)
    gen_matmul("matvec", 1, 64, 32)
    gen_matmul("square32", 32, 32, 32)
    gen_matmul("rect", 64, 128, 96)

    gen_conv2d("nopad", 3, 8, 8, 4, 3, 3, 1, 0)
    gen_conv2d("pad1", 3, 8, 8, 4, 3, 3, 1, 1)
    gen_conv2d("stride2", 8, 16, 16, 16, 3, 3, 2, 1)
    gen_conv2d("k1_odd", 4, 7, 9, 6, 1, 1, 1, 0)
    gen_conv2d("k5_pad2", 2, 6, 6, 3, 5, 5, 1, 2)

    print(f"\nwritten to {DATA_DIR}")


if __name__ == "__main__":
    main()
