# mini-infer

소형 CNN 추론 커널을 순수 C++로 구현하고 AVX2로 최적화하는 프로젝트 (PLAN.md 프로젝트 B).
현재 상태: 나이브 커널(matmul, Conv2D) + 벤치 하네스 + NumPy 레퍼런스 검증.

## 빌드 & 실행

```bash
python3 tests/gen_reference.py            # NumPy 테스트 벡터 생성 (tests/data/)
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

./build/test_correctness tests/data       # 정확성 검증 (NumPy 레퍼런스 대비)
./build/bench_naive results/naive_baseline.csv   # 벤치마크 → CSV
```

## 구조

- `src/naive/` — 나이브 커널. matmul(row-major), Conv2D(CHW, stride/pad 지원)
- `bench/` — 자체 벤치 하네스. 워밍업 10회 + 측정 100회, 중앙값/p99/min/GFLOP/s
- `tests/` — `gen_reference.py`가 float64 누적 NumPy 레퍼런스를 바이너리로 생성,
  `test_correctness`가 상대+절대 허용치 1e-3으로 비교
- `results/` — 벤치 CSV (빌드 플래그 포함, 재현용)

## 측정 규칙

단발 측정 금지. 항상 워밍업 제외 100회 이상 반복, 중앙값과 p99를 함께 기록한다.
