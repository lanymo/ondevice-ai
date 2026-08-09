#!/usr/bin/env bash
# cref/ 의 추론 커널·모델 헤더를 펌웨어 트리로 복사한다.
#
#   ./sync_firmware.sh          복사
#   ./sync_firmware.sh --check  차이만 보고 (CI/커밋 전 확인용, 아무것도 안 고침)
#
# **방향은 한쪽이다: cref → firmware.** cref 쪽이 원본이고, 거기서만 -O2 -Werror
# PC 빌드로 Python과 비트 단위 대조가 돌아간다(cref/test_ae_int8.c). 펌웨어 안의
# 것은 사본이다. 펌웨어 쪽을 고치면 다음 export에서 조용히 덮여 사라진다.
#
# 왜 심볼릭 링크나 -I 경로가 아니라 복사인가: CubeIDE가 생성하는 makefile이
# ../Core/Src/*.c 만 훑고, 프로젝트 밖 소스를 추가하면 .cproject 와 Debug/ 양쪽을
# 손으로 맞춰야 한다(CLAUDE.md). 그 상태가 더 잘 깨진다.

set -euo pipefail
cd "$(dirname "$0")"

SRC=cref
INC=../firmware/Core/Inc
SRCDIR=../firmware/Core/Src

HEADERS=(model_weights.h ae_infer.h ae_infer_f32.h ae_testvec_board.h)
SOURCES=(ae_infer.c ae_infer_f32.c)

check=0
[ "${1:-}" = "--check" ] && check=1

rc=0
copy_one() {
  local from=$1 to=$2
  if [ "$check" = "1" ]; then
    if ! diff -q "$from" "$to" >/dev/null 2>&1; then
      echo "다름: $to  (원본 $from)"
      rc=1
    fi
  else
    cp "$from" "$to"
    echo "  $to"
  fi
}

[ "$check" = "1" ] || echo "cref → firmware 동기화:"
for f in "${HEADERS[@]}"; do copy_one "$SRC/$f" "$INC/$f"; done
for f in "${SOURCES[@]}"; do copy_one "$SRC/$f" "$SRCDIR/$f"; done

if [ "$check" = "1" ]; then
  [ "$rc" = "0" ] && echo "펌웨어 사본이 cref와 일치한다."
  exit $rc
fi

cat <<'EOF'

새 .c 를 추가했다면 펌웨어 Debug/ 에도 등록해야 한다 (둘 다 gitignore됨):
  Debug/Core/Src/subdir.mk 의 C_SRCS / OBJS / C_DEPS
  Debug/objects.list
첫째만 하면 링크에서 undefined reference가 난다. (CLAUDE.md)
EOF
