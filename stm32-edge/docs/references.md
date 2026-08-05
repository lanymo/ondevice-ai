# 1차 자료 — 어디를 봐야 하는가

> **용도**: "이걸 어디서 확인하지?"에 답하는 색인. AI 없이 진행할 때, 그리고 AI가 준 답을
> **검증할 때** 보는 곳. CLAUDE.md의 *"검증 안 된 레지스터 주소/핀 번호를 단정하지 말 것"*이
> 실제로는 "이 문서들로 확인한다"는 뜻이다.
>
> 문서 번호는 st.com / invensense 제품 페이지의 Documentation 탭에서 검색하면 나온다.
> **번호가 기억 안 나면 제품명 + "reference manual"로 검색**하는 게 빠르다.

---

## 1. 질문 → 문서 매핑

| 알고 싶은 것 | 문서 | 비고 |
|---|---|---|
| I2C·TIM·RCC 레지스터, 클럭 트리, 인터럽트 벡터표 | **RM0383** — STM32F411xC/E Reference Manual | 1,000쪽+. **검색해서 해당 절만** |
| 핀 배치(AF 매핑), 전기적 특성, 최대 클럭, 패키지 | **STM32F411RE 데이터시트** | **RM과 다른 문서.** 초심자가 제일 헷갈리는 지점 |
| **DWT / CYCCNT, NVIC, BASEPRI, PendSV, SysTick** | **ARM** — Cortex-M4 Devices Generic User Guide (DUI 0553), ARMv7-M ARM (DDI 0403) | ⚠️ **ST 문서에 없다.** 코어는 ARM 소유. 이걸 모르면 RM0383에서 DWT를 영원히 못 찾음 |
| LD2가 왜 PA5? B1 버튼? ST-LINK 가상 COM 포트? 점퍼? | **UM1724** — STM32 Nucleo-64 boards | 보드 문서는 또 별개 |
| HAL 함수 인자·동작 | **UM1725** — STM32F4 HAL/LL drivers | 실제로는 **헤더 주석이 더 빠르고 정확**하다 |
| 칩 버그 | **Errata sheet** (제품 페이지에 별도) | 마지막 수단. 가끔 진짜 정답 |
| MPU6050 레지스터 (`PWR_MGMT_1` 0x6B, `ACCEL_XOUT_H` 0x3B, `WHO_AM_I` 0x75, `ACCEL_CONFIG`) | **RM-MPU-6000A-00** — Register Map and Descriptions | 레지스터는 **이쪽** |
| MPU6050 노이즈·대역폭·전류·온도 특성 | **PS-MPU-6000A-00** — Product Specification | 성능 수치는 **저쪽**. 2개 문서인 걸 몰라 헤맴 |
| FreeRTOS API, 우선순위/인터럽트 규칙 | freertos.org API Reference + **Mastering the FreeRTOS Real Time Kernel** (무료 PDF) | `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 설명이 여기 |
| 실시간 스케줄링 이론 | Liu & Layland (1973), Buttazzo *Hard Real-Time Computing Systems* | [realtime-scheduling.md](realtime-scheduling.md)에 정리됨 |
| 양자화 scale/zero-point, requantize | **Jacob et al. (2018)** — *Quantization and Training of Neural Networks for Efficient Integer-Arithmetic-Only Inference* / gemmlowp 문서 | 우리 [`ae_requantize`](../model/cref/ae_infer.c#L43)가 이 논문의 구현체 |
| CMSIS-NN 커널 (스트레치) | ARM CMSIS-NN repo + *CMSIS-NN* 논문 | `arm_fully_connected_s8` 규약에 이미 맞춰둠 |

---

## 2. 문서보다 자주 보는 것 (실무 순서)

1. **ST 공식 예제 코드** — `STM32Cube_FW_F4` 패키지의 `Projects/` 폴더.
   "TIM 인터럽트 어떻게 켜지"는 매뉴얼 읽는 것보다 예제 하나 여는 게 10배 빠르다.
   실무에서 실제로 여기서 시작한다.
2. **CubeMX가 생성한 코드** — 클럭 설정([system_stm32f4xx.c](../firmware/Core/Src/system_stm32f4xx.c)),
   MSP 초기화([stm32f4xx_hal_msp.c](../firmware/Core/Src/stm32f4xx_hal_msp.c)). 읽으면 관례가 보인다.
3. **HAL 헤더 주석** — `stm32f4xx_hal_i2c.h` 등. UM1725보다 최신이고 정확하다.
4. **ST Community 포럼** — 증상은 있는데 원인을 모를 때.

> **AI가 대체한 건 지식이 아니라 탐색 시간**이다. RM0383에서 TIM 프리스케일러 공식을 찾는 데
> 예전엔 20분, 지금은 20초. 하지만 **"타이머 클럭이 왜 PCLK×2인가"**는 어차피 문서를 봐야 알고,
> 그건 지금도 같다. 그래서 이 색인이 필요하다.

---

## 3. 이 리포에서 "문서로 확인해야 하는데 아직 안 한 것"

| 항목 | 현재 상태 | 볼 문서 |
|---|---|---|
| **가속도/자이로 풀스케일 레인지** | **가정만 함** — [PLAN.md W3](../../PLAN.md) 참조 | RM-MPU-6000A (`ACCEL_CONFIG`/`GYRO_CONFIG`) |
| I2C 클럭 100kHz의 근거 | 관례로 설정 | RM0383 I2C 절 + MPU6050 PS (400kHz Fast Mode 지원 여부) |
| MPU6050 DLPF(저역통과 필터) 설정 | **미설정** — 리셋 기본값 사용 | RM-MPU-6000A `CONFIG`(0x1A). 100Hz 샘플링에 적절한 대역폭인가? |

세 번째는 진동 감지에서 **에일리어싱**과 직결된다 — 샘플링 100Hz면 나이퀴스트 50Hz인데
센서 내부 대역폭이 그보다 넓으면 고주파가 접혀 들어온다. ([timeseries-windowing.md](timeseries-windowing.md))
