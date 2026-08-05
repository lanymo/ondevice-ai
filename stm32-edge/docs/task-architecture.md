# W2 태스크 구조 — 4태스크 + 마감시한 이벤트 경로

> 구현: [Core/Src/app_tasks.c](../firmware/Core/Src/app_tasks.c) · [Core/Inc/app_tasks.h](../firmware/Core/Inc/app_tasks.h)
> 선행 개념: freertos-concepts.md, [measurement-methodology.md](measurement-methodology.md)
>
> W2 DoD: "추론 없이 **마감시한 이벤트 → 로컬 액션 지연**이 UART로 찍힘 + IMU 100Hz 스트림".
> 이 문서는 그 구조와, 보드에서 무엇을 확인해야 하는지를 적는다.

---

## 1. 태스크 4개와 우선순위

| 태스크 | 우선순위 | 스택(워드) | 주기/트리거 | 하는 일 |
|---|---|---|---|---|
| `action` | **48** | 256 | TIM2 이벤트 (50ms) | 지연 측정 + LD2 토글 = **로컬 액션** |
| `sensor` | 40 | 384 | 10ms (`vTaskDelayUntil`) | MPU6050 6축 폴링 100Hz + 주기 측정 |
| `log` | 16 | 512 | 큐 수신 | printf/CSV 출력 **전담** |
| `infer` | 8 | 192 | 20ms busy + 30ms 휴지 | W2에선 추론 **스텁**(CPU 부하 40%) |

우선순위 숫자가 클수록 높다(`configMAX_PRIORITIES=56`). 배치의 근거:

- **action이 최상위**: 하드 마감시한을 가진 유일한 태스크. 다른 무엇도 이걸 밀어내면 안 된다.
- **infer가 최하위**: 무거운 payload지만 마감시한이 없다. W2 베이스라인은 "추론 < 액션"인
  정상 배치이고, **W3에서 이 관계를 뒤집어(추론 > 액션) 액션 지연 p99·max가 얼마나 나빠지는지**
  재는 것이 축 A 실험의 핵심이다.
- **log가 infer보다 위**: UART 출력이 추론 부하에 막혀 밀리면 측정 보고가 실시간성을 잃는다.
  대신 action/sensor보다는 아래 — printf가 마감시한 경로를 절대 물지 않게.

`printf`는 **로그 태스크에서만** 부른다. `HAL_UART_Transmit`은 블로킹이라 115200bps에서
한 줄에 수 ms가 걸린다. 이걸 action 태스크에서 부르면 측정 대상 자체가 오염된다
(probe effect). 측정값은 큐로 넘기고 통계 계산·출력은 전부 로그 태스크 몫이다.

CubeMX가 만든 `defaultTask`는 `osThreadExit()`로 즉시 종료시킨다. 실제 태스크 4개는
전부 `xTaskCreateStatic` — **동적 할당 없음**(CLAUDE.md 규칙).

---

## 2. 마감시한 이벤트 → 액션 경로

```
TIM2 update ISR ──(1) t0 = DWT->CYCCNT
       │
       └──(2) vTaskNotifyGiveFromISR(action) + portYIELD_FROM_ISR
                     │
                     └──(3) action 태스크 기상: t1 = DWT->CYCCNT
                                  지연 = t1 - t0        ← 측정값
                            (4) HAL_GPIO_TogglePin(LD2) ← 로컬 액션
```

- **이벤트 소스 = TIM2**(내부 전용, 핀 없음, 50ms 주기). 버튼이 아니라 타이머인 이유는
  **결정론과 재현성** — 100회 반복 통계를 사람 손으로 만들 수 없다. (B1 버튼은 예비.)
- ISR은 `HAL_TIM_IRQHandler`를 거치지 않고 UIF만 직접 지운다. HAL 디스패치가 끼면
  그 오버헤드가 측정값에 섞인다. ISR이 하는 일은 **타임스탬프 + 노티피케이션** 둘뿐.
- ISR 우선순위 6 — `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`(5)보다 **낮게**(숫자 크게)
  잡아야 ISR에서 `...FromISR` API를 부를 수 있다. 이걸 어기면 `configASSERT`에 걸린다.
- 이 지연에 포함되는 것: ISR 진입/처리 + 컨텍스트 스위치 + 스케줄러 오버헤드.
  포함되지 않는 것: 없음(액션 직전까지 전부). 즉 **"이벤트가 났는데 실제로 반응하기까지"**의
  전량이고, 그래서 W3에서 우선순위를 흔들면 이 값이 움직인다.

---

## 3. 측정 규율 (CLAUDE.md)

- **워밍업 16샘플 제외** 후 통계 시작.
- **배치**: 액션 128샘플(≈6.4초), 센서 256샘플(≈2.56초). 배치마다 min/중앙값/p99/max를 출력.
  단발 측정 없음.
- **더블 버퍼 + 소유권 플래그**: 로그 태스크가 한쪽 버퍼를 정렬(`dwt_stats`는 제자리 정렬 =
  파괴적)하는 동안 생산자는 반대쪽에 쌓는다. `busy` 플래그로 소유권을 표시해서, 로그가
  밀렸을 때 생산자가 두 바퀴 돌아 정렬 중인 버퍼를 덮어쓰는 일을 막는다.
  이때 빠진 샘플은 `skip=`으로 보고 — **조용히 버리지 않는다.**
- 원본 단위는 항상 **사이클**. µs는 84MHz(HSI, ±1%) 기준 환산치이므로 절대시간 정밀도는 없다.
- **스택 오버플로 훅**(`configCHECK_FOR_STACK_OVERFLOW=2`): 태스크 스택 끝에 패턴을 깔고
  전환마다 검사한다. 걸리면 `vApplicationStackOverflowHook`이 인터럽트를 끄고 **LD2를
  1초 주기로 토글** — 정상(50ms 토글)과 눈으로 즉시 구분된다. 훅 안에서 printf는 부르지
  않는다(이미 스택이 깨진 상태라 더 깊이 들어가면 원인 정보까지 잃는다).
  잡힌 태스크 이름은 전역 `g_stack_overflow_task`에 남는다.

출력에 같이 찍히는 카운터:

| 필드 | 뜻 | 정상값 |
|---|---|---|
| `miss(1ms)` | 지연이 잠정 마감시한 1ms를 넘은 횟수 | **0** |
| `lost` | 액션이 못 따라가 이벤트가 합쳐진 횟수 | **0** |
| `skip` | 로그 백프레셔로 측정에서 빠진 샘플 | 0 (드물게 >0이어도 무해) |
| `fail` | I2C 읽기 실패 횟수 | **0** |

---

## 4. 보드에서 확인할 것 (플래시 후)

UART 115200 8N1. 부팅 시 기존 브링업(DWT selftest / I2C 스캔 / MPU6050) 로그가 먼저 나오고,
스케줄러가 시작되면 아래가 반복된다.

```
=== W2 tasks up: action(48) sensor(40) log(16) infer(8) ===
event=50ms deadline=1ms batch: act=128 sen=256 warmup=16
tag,n,min,median,p99,max
[SEN] batch=0 n=256 interval_us med=10000 p99=... max=... fail=0 skip=0 win=... a=(...) g=(...)
imu_interval_task,256,...,...,...,...
[ACT] batch=0 n=128 lat_us med=... p99=... max=... miss(1ms)=0 lost=0 skip=0
action_latency,128,...,...,...,...
```

체크리스트:

1. **LD2(초록 LED)가 50ms마다 토글** → 육안으로는 10Hz 깜빡임. 이게 "로컬 액션"의 시각 증거.
2. `[ACT]` 줄의 `med`/`p99`/`max`가 µs 단위로 찍히는가 → **W2 DoD의 핵심 수치**.
   `max`가 `med`보다 크게 튀는지 함께 볼 것(선점·ISR 개입의 흔적).
3. `miss(1ms)=0`, `lost=0` — 하나라도 0이 아니면 마감시한 설정이나 부하 가정을 재검토.
4. `[SEN]`의 `interval_us med`가 **10000 근처**인가 → 태스크 환경에서도 100Hz 유지.
   `fail=0`이어야 I2C가 건강한 것.
   (W2 초기 `HAL_Delay` 방식은 11.00ms = 90.9Hz였다 — `dwt_baseline.csv`의
   `imu_interval_haldelay`. 이번 값이 10000에 붙어야 그 문제가 해결된 것.)
5. `a=(x,y,z)`가 보드를 **기울이면** 변하고, `g=(x,y,z)`가 보드를 **돌리면** 변하는가
   → 가속도·자이로 둘 다 살아있다는 생존 신호. 정지 상태에서 자이로는 0 근처
   (바이어스만큼 치우칠 수 있음).
6. `win=`이 계속 증가하는가 → 추론 입력 윈도우가 hop 64마다 쌓이는 중.
   100Hz·hop 64면 **0.64초마다 1개**씩 늘어야 정상.

수치를 `measurements/action_latency.csv`에 옮겨 적을 때는 CSV 줄(`action_latency,...`)을
그대로 쓰고 `clock_hz,measured_at,note` 열을 채운다 ([dwt_baseline.csv](../measurements/dwt_baseline.csv)와 동일 규약).

---

## 5. W3로 넘어가는 접점

- `s_win[]`에 센서 태스크가 이미 데이터를 쌓고 있다. **6축 raw int16 × 128샘플 = 1536B**로,
  `model/config.py`의 `CHANNELS(ax,ay,az,gx,gy,gz)` / `WIN_LEN=128` / `HOP=64`와 맞췄다.
  이게 어긋나면 학습된 특징 분포와 온보드 특징이 다른 것을 재게 된다.
  g/dps 환산(1g=16384, 1dps=131 LSB)과 24차원 특징 추출은 윈도우를 뽑는 시점에 한다.
  `s_win_ready`가 hop마다 증가하므로 W3에서는 여기서 추론 태스크를 깨우면 된다.
  (timeseries-windowing.md)
- `prv_infer_task`의 busy-wait 스텁을 실제 int8 순전파로 갈아끼우면 축 B가 붙는다.
- 우선순위 상수(`PRIO_*`)와 청킹 여부가 축 A 스윕의 조작 변수다:
  (A) 추론 > 액션 / (B) 추론 < 액션 / (C) monolithic / (D) 청킹.
  **W2 베이스라인 = (B) + (C)**.
