# PC에서 순수 C 추론을 검증한다. 보드/CubeIDE 없이 돌아간다.
#
#   make        빌드
#   make test   testvec.bin 대조 실행
#
# ae_infer.c / ae_infer.h / model_weights.h 는 W3에서 그대로 펌웨어로 간다.
# 여기서 -Werror로 조여두면 CubeIDE 쪽에서 새로 터질 경고가 줄어든다.

CC      ?= gcc
CFLAGS  := -std=c99 -O2 -Wall -Wextra -Werror -Wconversion -Wshadow \
           -Wstrict-prototypes -Wpointer-arith
LDLIBS  := -lm

TARGET  := test_ae_int8
OBJS    := ae_infer.o test_ae_int8.o

.PHONY: all test clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

ae_infer.o: ae_infer.c ae_infer.h model_weights.h
test_ae_int8.o: test_ae_int8.c ae_infer.h model_weights.h

test: $(TARGET) testvec.bin
	./$(TARGET) testvec.bin

testvec.bin model_weights.h:
	@echo "없음: $@ — 먼저 'python3 ../export_header.py' 를 돌려라" && false

clean:
	rm -f $(TARGET) $(OBJS)
