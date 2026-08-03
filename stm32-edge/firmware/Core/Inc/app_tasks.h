/**
  ******************************************************************************
  * @file    app_tasks.h
  * @brief   W2 4태스크 골격: 센서폴링 / 추론(스텁) / 로컬액션 / 로깅
  *
  * 우선순위 설계 (숫자 클수록 높음, configMAX_PRIORITIES=56):
  *   action(48) > sensor(40) > [defaultTask 24, 즉시 종료] > log(16) > infer(8) > idle(0)
  *
  * W2 베이스라인 = "추론이 액션보다 낮은" 정상 배치. W3 스케줄링 스윕에서
  * 이 우선순위/청킹을 흔들어 액션 지연 p99/max 변화를 잰다.
  *
  * 이벤트 경로: TIM2 ISR(50ms 주기, DWT 타임스탬프) → 태스크 노티피케이션
  *   → action 태스크 기상(t1) → 지연 = t1 - t0 → LD2 토글(로컬 액션).
  * printf는 로깅 태스크 전용 (UART 블로킹이 다른 태스크를 못 물게).
  ******************************************************************************
  */
#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  4태스크 + 로그 큐 + (action 태스크 안에서) TIM2를 생성한다.
  *         전부 정적 할당 - FreeRTOS 힙/malloc 안 씀.
  * @note   MX_FREERTOS_Init()의 USER CODE RTOS_THREADS에서 호출.
  */
void app_tasks_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASKS_H */
