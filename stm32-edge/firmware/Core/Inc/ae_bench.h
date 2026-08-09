/**
  ******************************************************************************
  * @file    ae_bench.h
  * @brief   부팅 시 1회: 추론 자가검증 + float32 vs int8 커널 사이클 측정
  *
  * 두 가지를 한다. 순서에 의미가 있다.
  *
  *   [1] 자가검증 — 보드의 추론이 PC(Python)와 **같은 답**을 내는가.
  *       정수 경로는 비트정확이어야 한다. 이게 깨진 채로 아래 사이클을 재봐야
  *       "틀린 커널이 얼마나 빠른가"를 재는 것이라 의미가 없다.
  *
  *   [2] 커널 비교 — docs/system-design.md §5. 순수 C int8이 M4F의 FPU float보다
  *       빠른가? 일반론("int8이 4배 빠르다")은 SIMD를 전제하는데 우리는 순수 C다.
  *       결과가 어느 쪽이든 그대로 measurements/kernel_compare.csv에 남긴다.
  *
  * **스케줄러 시작 전에 부를 것.** 태스크 선점이 없어야 커널 자체의 비용이 나온다
  * (HAL 틱 TIM10 인터럽트는 남아 있어서 꼬리에 섞인다 - 그래서 min도 같이 본다).
  * 태스크 환경에서의 실제 추론 소요는 별개로 app_tasks.c의 추론 태스크가 잰다.
  ******************************************************************************
  */
#ifndef __AE_BENCH_H
#define __AE_BENCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
  * @brief  자가검증 + 커널 사이클 측정을 수행하고 결과를 UART에 찍는다.
  * @retval 자가검증 통과 여부. false여도 측정은 수행한다(진단 정보가 더 많은 쪽).
  */
bool ae_bench(void);

#ifdef __cplusplus
}
#endif

#endif /* __AE_BENCH_H */
