#!/usr/bin/env bash
# 파이프라인 전체를 처음부터 끝까지 돌린다. clone 직후 이거 하나면 재현된다.
#
#   ./run_all.sh
#
# 시드가 고정돼 있어 매번 같은 숫자가 나온다. 실측 데이터를 쓸 거면
# SKIP_SYNTH=1 을 주고 data/ 에 CSV + manifest.csv 를 직접 채워 넣을 것.

set -euo pipefail
cd "$(dirname "$0")"

step() { printf '\n\033[1m=== %s ===\033[0m\n' "$1"; }

if [ "${SKIP_SYNTH:-0}" = "1" ]; then
  step "1/6 합성 데이터 생성 — 건너뜀 (SKIP_SYNTH=1, data/ 의 실측 CSV 사용)"
else
  step "1/6 합성 데이터 생성 (보드 실측의 임시 대역)"
  python3 synth_data.py
fi

step "2/6 float32 오토인코더 학습 + 임계값 산정"
python3 train.py --quiet

step "3/6 float vs int8 정확도 → measurements/accuracy.csv"
python3 evaluate.py

step "4/6 model_weights.h + 테스트벡터 export"
python3 export_header.py

step "5/6 순수 C 순전파 비트정확 검증 (펌웨어에 올릴 코드 그대로)"
make -C cref clean >/dev/null
make -C cref test

step "6/6 그래프 → measurements/plots/"
python3 plot.py

printf '\n\033[1m완료.\033[0m 산출물:\n'
printf '  cref/model_weights.h            펌웨어가 컴파일해 넣을 유일한 파일\n'
printf '  ../measurements/accuracy.csv    float vs int8 정확도\n'
printf '  ../measurements/plots/*.png     그래프 4장\n'