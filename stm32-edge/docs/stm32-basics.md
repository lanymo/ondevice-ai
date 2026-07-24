# STM32 기초 노트 — GPIO/UART/HAL/CubeMX (임베디드 초심자용)

> W1 화요일 태스크 자료. 보드 없이 진행 가능: §6의 CubeMX 실습은 CubeIDE만 있으면 되고,
> 빌드까지 확인할 수 있다. 보드가 도착하면 §7의 printf 코드를 그대로 쓴다.
> 선행 문서: freertos-concepts.md (RTOS는 이 문서의 내용 *위에* 올라가는 층이다).

---

## 1. STM32가 뭔가 — 이름부터 해부

STM32는 ST마이크로일렉트로닉스가 만드는 32비트 MCU 제품군이다. 우리가 쓰는 칩
이름을 뜯어보면:

```
STM32 F4 11 R E
│     │  │  │ └─ 플래시 크기: E = 512KB
│     │  │  └─── 핀 수: R = 64핀
│     │  └────── 라인 넘버 (F4 패밀리 안의 세부 모델)
│     └───────── 패밀리: F4 = 고성능군, Cortex-M4 코어
└─────────────── ST의 32비트 MCU 브랜드
```

- 코어는 ARM **Cortex-M4F** (100MHz, F는 FPU = float 하드웨어 연산 지원).
  ARM이 코어 설계도를 팔고, ST가 그 코어에 자기네 주변장치를 붙여 칩으로 만든다.
  그래서 "ARM 코어 지식"은 삼성/NXP/TI 칩에도 통하고, "ST 주변장치 지식"은 STM32 안에서 통한다.
- RAM 128KB, 플래시 512KB. 펌웨어는 플래시에 구워지고, 전원이 켜지면 거기서 실행 시작.

### Nucleo 보드는 뭔가

칩만 사면 납땜 없이는 아무것도 못 한다. **Nucleo-F411RE는 STM32F411RE 칩 +
전원 회로 + 디버거를 한 판에 얹은 개발보드**다. 보드를 보면 두 부분으로 나뉜다:

```
┌─────────────────┐
│  ST-LINK 부분    │ ← 미니 USB 커넥터가 있는 위쪽. 별도의 작은 MCU가
│  (위쪽, 분리선 有)│    "디버거" 역할: PC와 타겟 칩 사이의 통역사
├ ─ ─ ─ ─ ─ ─ ─ ─┤
│  타겟 보드 부분   │ ← STM32F411RE 본체 + 핀헤더 + 유저 LED(LD2) +
│  (아래쪽)        │    유저 버튼(B1, 파란색)
└─────────────────┘
```

USB 케이블 하나 꽂으면 세 가지가 동시에 된다:
1. **전원 공급** (보드 단독 테스트 시 배터리 불필요)
2. **플래싱/디버깅** — 컴파일한 펌웨어를 칩에 굽고, 중단점 걸고 변수 들여다보기
3. **가상 시리얼 포트(VCP)** — 칩의 UART가 USB를 타고 PC에 COM 포트로 나타남.
   printf 디버깅의 통로가 바로 이것.

## 2. 부팅부터 main까지 — OS가 없다는 것의 의미

PC에선 부트로더→OS→프로세스 순서로 한참을 거치지만, MCU는:

1. 전원 인가 → 칩이 플래시의 정해진 주소에서 초기 스택 포인터와 리셋 핸들러 주소를 읽음
2. 리셋 핸들러(스타트업 코드, CubeIDE가 자동 생성하는 어셈블리)가 전역변수 초기화
3. `main()` 호출 — **여기서부터 끝까지 내 코드가 CPU를 독점한다**

`main()`이 리턴하면 갈 곳이 없다. 그래서 모든 펌웨어의 `main()`은 무한 루프로 끝난다.
(FreeRTOS를 쓰면 이 무한 루프 자리에 스케줄러 시작이 들어가는 것.)

## 3. 핵심 멘탈 모델: 주변장치 = 메모리에 매핑된 스위치판

MCU 프로그래밍의 본질은 이거 하나다: **칩 안의 모든 주변장치(GPIO, UART, 타이머...)는
특정 메모리 주소에 있는 레지스터(32비트 값)로 조종한다.** 예를 들어 "이 주소의
5번 비트에 1을 쓰면 PA5 핀에 3.3V가 나온다"는 식. 이걸 memory-mapped I/O라 한다.

```c
// 이론상 이렇게 생짜로도 제어 가능 (실제로 이렇게 안 쓰지만 원리 이해용)
*(volatile uint32_t*)0x40020014 |= (1 << 5);   // 어떤 주소의 5번 비트 셋
```

- 어느 주소에 뭐가 있는지는 칩의 **레퍼런스 매뉴얼**(F411은 RM0383, 800쪽짜리)에 다 있다.
- 이 주소들을 매번 찾아 쓰는 건 고통이므로, ST가 함수로 감싸서 제공하는 게 **HAL**이다.

### 핀 이름 읽는 법

STM32 핀은 `P + 포트문자 + 번호`다: PA5 = 포트 A의 5번 핀. 포트 하나(A, B, C...)가
핀 16개 묶음이고, 포트별로 GPIO 레지스터 세트가 하나씩 있다. 한 핀은 설정에 따라
GPIO도 되고 UART TX도 되고 PWM 출력도 된다 — 이 "겸직"을 **대체 기능(Alternate
Function, AF)**이라 하고, 핀맵 설계(docs/pinmap.md에 정리할 것)란 결국 "어느 핀에게
어느 직업을 줄까"를 정하는 일이다.

## 4. GPIO — 가장 단순한 주변장치

GPIO(General Purpose Input/Output) = 핀 하나를 1비트 입출력으로 쓰는 것.
핀마다 모드를 정한다:

| 모드 | 뜻 | 이 프로젝트에서의 예 |
|---|---|---|
| Output | 내가 0V/3.3V를 내보냄 | LED, L298N 방향 제어(IN1~IN4) |
| Input | 외부 전압을 읽음 (0 또는 1) | 유저 버튼, TCRT5000 라인센서 DO |
| Alternate Function | 다른 주변장치에게 핀을 양보 | PA2를 UART TX로, 타이머 채널을 PWM으로 |
| Analog | ADC로 연속값을 읽음 | (당장 계획엔 없음) |

입력 핀에는 풀업/풀다운 개념이 있다: 아무것도 연결 안 된 입력 핀은 전압이 떠서(floating)
0인지 1인지 무작위로 읽힌다. 내부 풀업 저항을 켜면 "평소엔 1, 눌리면 0"처럼 기본값을
정해줄 수 있다. (W2에서 I2C 풀업 얘기가 나올 때 다시 만난다.)

HAL로 쓰면 이렇게 된다 (코드 리딩용 — 함수 이름만 봐도 뜻이 통하는 게 HAL의 장점):

```c
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);    // PA5에 3.3V
HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);                 // 반전 (LED 깜빡임)
GPIO_PinState b = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); // PC13 읽기
```

Nucleo-64 보드의 관례(보드 매뉴얼 UM1724 기준, §6 실습에서 CubeMX가 자동 라벨링해주는
것으로 재확인할 것): 유저 LED **LD2 = PA5**, 유저 버튼 **B1 = PC13**.

## 5. UART — 디버깅의 생명줄

UART = 두 가닥 선(TX 송신, RX 수신)으로 바이트를 주고받는 가장 원시적인 시리얼 통신.
합의된 속도(**보드레이트**, 우리는 115200bps)로 비트를 쏘면 상대가 같은 속도로 샘플링해
읽는다. 클럭 선이 따로 없어서 양쪽 속도 설정이 다르면 글자가 깨진다(문자 깨짐의 90%가
보드레이트 불일치).

왜 생명줄인가: MCU엔 화면도 콘솔도 없다. **UART로 printf를 쏘고 PC 터미널로 보는 게
유일한 "출력창"**이다. W1 DoD가 "UART printf 성공"인 이유 — 이게 안 되면 이후 모든
디버깅이 장님이 된다.

Nucleo에서는 칩의 **USART2**(PA2=TX, PA3=RX)가 보드 배선으로 ST-LINK에 연결돼 있어서,
USB만 꽂으면 PC에서 시리얼 포트로 보인다. 별도 배선 없음. HAL 사용은:

```c
uint8_t msg[] = "hello\r\n";
HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 100);  // 100 = 타임아웃(ms)
```

`huart2`는 CubeMX가 생성해주는 "USART2 설정 묶음" 구조체(핸들)다. HAL의 모든 주변장치가
이 핸들 패턴을 쓴다: 설정은 구조체에, 동작은 `HAL_XXX_동사(&핸들, ...)` 함수로.

## 6. CubeMX 실습 — 보드 없이 여기까지 해볼 것 (30~60분)

CubeMX는 CubeIDE에 내장된 그래픽 설정 도구다. 핀맵을 클릭으로 정하면 초기화 C 코드를
생성해준다. 실습 순서:

1. CubeIDE → File → New → **STM32 Project**
2. 타겟 선택 창에서 **Board Selector 탭** → "NUCLEO-F411RE" 검색 → 선택.
   (칩이 아니라 보드로 시작하면 LD2, B1, USART2-VCP 연결이 자동 설정된다 —
   §4~5의 핀 배정이 맞는지 여기서 눈으로 확인)
3. "Initialize all peripherals in default mode?" → Yes
4. **Pinout 뷰 구경**: 칩 그림에서 PA5(LD2), PC13(B1), PA2/PA3(USART2) 라벨 확인.
   아무 핀이나 클릭해서 어떤 대체 기능들이 가능한지 목록 훑어보기 — W2 핀맵 설계 때
   할 일이 정확히 이 화면에서 벌어진다
5. 왼쪽 트리 Connectivity → USART2: Mode = Asynchronous, 115200 8N1인지 확인
6. 저장(Ctrl+S) → 코드 생성됨 → **읽기 투어**:
   - `Core/Src/main.c` — `HAL_Init()` → `SystemClock_Config()` → `MX_GPIO_Init()` →
     `MX_USART2_UART_Init()` → `while(1)`. 이 뼈대가 모든 STM32 프로젝트의 표준 형태
   - `main.c`의 `/* USER CODE BEGIN x */ ... END */` 주석 쌍 — **이 사이에 쓴 코드만
     CubeMX 재생성 시 살아남는다.** 밖에 쓰면 다음 코드 생성 때 삭제됨 (최다 사고 포인트)
   - `Core/Src/stm32f4xx_hal_msp.c` — "PA2를 USART2에게 AF로 배정"하는 저수준 설정이
     여기 있다. HAL이 §3의 레지스터 조작을 어떻게 감싸는지 보이는 파일
   - `.ioc` 파일 — CubeMX 설정 자체. 이것도 git에 커밋하는 파일이다
7. 망치 아이콘으로 **빌드** — 보드 없어도 컴파일은 된다. "Build Finished. 0 errors"와
   메모리 사용 요약(text/data/bss)이 나오면 성공

## 7. 보드 도착하면 바로 쓸 것 — printf 리타겟

`printf`가 UART로 나가게 하려면 표준 라이브러리의 출력 함수를 UART로 연결해야 한다.
`main.c`의 `/* USER CODE BEGIN 0 */` 블록에:

```c
/* USER CODE BEGIN 0 */
#include <stdio.h>

// printf가 내부적으로 호출하는 _write를 UART2로 리타겟 (gcc/newlib 기준)
int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 0 */
```

`while(1)` 안의 `/* USER CODE BEGIN 3 */`에:

```c
    printf("hello from F411RE (%lu ms)\r\n", HAL_GetTick());
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(500);
```

보드 연결 → Run 버튼(플래시+실행) → CubeIDE 하단 터미널 또는 PC 시리얼 터미널에서
해당 포트를 115200으로 열기 → 0.5초마다 문장 출력 + LED 깜빡임 = **W1 DoD 달성**.
주의: float를 printf로 찍으려면 프로젝트 설정에서 `-u _printf_float` 링커 플래그가
필요하다 (기본 비활성, 필요해질 때 켜기).

## 8. HAL 한 겹 더 — "OS를 C로 짜던 감각"과 잇기

§3에서 "주변장치 = 메모리에 매핑된 레지스터"라 했다. **HAL은 그 레지스터 조작을 C 함수로
감싼 라이브러리**다. 네가 "이거 C로 OS 짤 때랑 비슷한데?"라고 느낀 건 착각이 아니라 정확한
직관이다 — 이 절이 그 이유를 짚는다.

### 세 층위: 레지스터 → LL → HAL

같은 "PA5를 켠다"를 세 높이로 쓸 수 있다:

| 층 | 무엇 | 예 | 성격 |
|---|---|---|---|
| 레지스터 직접 | CMSIS 헤더의 구조체/매크로 | `GPIOA->BSRR = (1<<5);` | §3의 생짜 포인터에 이름만 붙인 것. 가장 빠름/날것 |
| LL (Low-Layer) | 얇은 인라인 래퍼, 레지스터 1:1 | `LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);` | 빠르고 얇음 |
| **HAL** | 상태·에러·타임아웃까지 관리하는 두꺼운 래퍼 | `HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);` | **우리가 주로 씀.** 편하지만 오버헤드 있음 |

셋은 배타적이지 않다. 평소 HAL을 쓰다가, 사이클이 아까운 핫패스(측정 대상)만 레지스터로
내려가는 식으로 섞는다.

### 핸들 패턴 — §5의 `huart2`가 다시 나오는 이유

HAL의 모든 주변장치는 **"설정은 구조체에, 동작은 함수에"** 패턴이다:

```c
UART_HandleTypeDef huart2;                       // 이 UART의 상태 전부(인스턴스, 보드레이트, 버퍼...)
HAL_UART_Transmit(&huart2, buf, len, 100);       // 동작은 핸들을 넘겨서
```

> CS 연결: 이건 **C로 하는 객체지향**이다. 구조체가 객체, 핸들 포인터가 `this`. Pintos에서
> `struct thread *`, `struct lock *`를 함수마다 넘기던 것과 **완전히 같은 패턴**이다.

### 이름 규칙 & 반환값

- **`HAL_<주변장치>_<동사>()`**: `HAL_GPIO_WritePin`, `HAL_I2C_Mem_Read`, `HAL_TIM_IC_Start_IT`.
  함수 이름만 봐도 뜻이 통하는 게 HAL의 장점.
- 반환은 대개 `HAL_StatusTypeDef` = `HAL_OK / HAL_ERROR / HAL_BUSY / HAL_TIMEOUT`.
  **체크하는 습관을 들여라** — I2C 무응답 같은 걸 여기서 잡는다.
- 블로킹 함수엔 **타임아웃(ms)** 인자가 있다. 무한정 안 걸리게 하는 안전장치.

### 한 가지 일, 세 가지 방식 — 블로킹 / 인터럽트 / DMA (이 프로젝트에 중요)

HAL 함수엔 보통 세 변형이 있고, **이 선택이 곧 실시간 지터에 영향**을 준다:

- `HAL_UART_Transmit(...)` — **블로킹(폴링)**. 다 보낼 때까지 CPU가 그 자리서 대기.
  로거가 이걸 쓰면 그동안 태스크가 묶여 지터의 원인이 된다.
- `HAL_UART_Transmit_IT(...)` — **인터럽트 기반**. 보내고 즉시 리턴, 완료 시 콜백
  (`HAL_UART_TxCpltCallback`)이 불린다. CPU가 자유로워짐.
- `HAL_UART_Transmit_DMA(...)` — **DMA**가 CPU 개입 없이 메모리→주변장치로 옮김. 대량 전송에 최적.

→ [freertos-concepts.md](freertos-concepts.md) §4의 deferred interrupt 패턴과 직결. MPU6050 I2C,
HC-SR04 인풋캡처도 `_IT` 변형을 써서 "ISR은 짧게, 계산은 태스크로" 간다.

### volatile — 왜 반드시 필요한가 (§3 되짚기)

레지스터 값은 **하드웨어가 언제든 바꾼다**(센서가 데이터를 넣고, 타이머가 카운트를 올리고).
컴파일러가 "이 변수 안 바뀌네" 하고 레지스터에 캐싱하거나 읽기를 없애버리면 오동작한다.
그래서 CMSIS는 모든 주변장치 레지스터를 `volatile`로 정의해둔다.

> **네가 직접 메모리 매핑 포인터를 만들 땐 `volatile`이 필수다** (§3의 `*(volatile uint32_t*)addr`).
> Pintos에서 디바이스 레지스터·공유 변수에 volatile 신경 쓰던 것과 똑같은 이유.

### Pintos(OS를 C로) ↔ STM32(HAL) 대비 — 네가 느낀 "비슷함"의 정체

| | Pintos (x86, OS 과제) | STM32 / HAL (베어메탈·RTOS) |
|---|---|---|
| 하드웨어 접근 | `inb`/`outb` 포트 I/O + 일부 MMIO, volatile | **순수 메모리 매핑 I/O** (ARM엔 포트 공간이 없음), volatile |
| 추상화 | 직접 매크로/인라인 | HAL 함수 래퍼(이름 붙은 드라이버) |
| 인터럽트 | 핸들러 등록, `intr_disable()` | ISR + `__disable_irq()`, NVIC |
| 그 아래 OS | **없음 — 네가 OS다** | 없음(베어메탈) 또는 FreeRTOS(네가 얹음) |
| 메모리 | 페이지/palloc, 네가 관리 | 정적/아레나, malloc 금지 |
| 표준 라이브러리 | 최소(직접 구현) | newlib 일부, printf는 리타겟 필요(§7) |

핵심 공통점: **밑에 아무도 없다.** 파일·소켓·스레드를 OS가 떠먹여 주는 게 아니라,
하드웨어 레지스터를 직접 건드려 **그 서비스를 네가 만든다.** HAL은 그 레지스터 poking에
이름표를 붙여준 것뿐 — 그래서 명령어들이 "C로 OS 짜던 것"과 닮게 느껴지는 게 맞다.

### 앱 C와 다른 점 (착각 주의)

- `HAL_Delay(500)`은 sleep이 아니라 **바쁜 대기에 가깝다**(틱 카운트를 돌며 기다림). RTOS에선
  `osDelay`/`vTaskDelay`로 대체해야 CPU를 양보한다([freertos-concepts.md](freertos-concepts.md) §2).
- 힙·파일·네트워크·프로세스가 없다. `printf`는 UART 리타겟(§7) 없이는 **아무 데도 안 나간다.**
- 크래시해도 OS가 안 잡아준다 — `HardFault_Handler`로 떨어진다(그래서 UART 로그가 생명줄).

---

## 9. 자기 점검 질문

1. Nucleo 보드에서 USB 케이블이 하는 세 가지 역할은?
2. "PA5에 1을 쓴다"는 문장을 §3의 멘탈 모델로 다시 말하면?
3. 한 핀이 GPIO도 되고 UART TX도 될 수 있는 메커니즘의 이름은?
4. UART 출력이 `?%$#@` 같은 깨진 글자로 보인다. 첫 번째 용의자는?
5. CubeMX로 코드를 재생성했더니 내가 쓴 코드가 사라졌다. 뭘 어겼나?
6. 이 문서의 GPIO/UART 위에 freertos-concepts.md의 태스크가 올라간다.
   "UART 로깅"이 별도 태스크인 이유를 두 문서를 엮어 설명하면?

---

### 더 읽을 것 (필요할 때만)

- UM1724 — Nucleo-64 보드 매뉴얼 (LED/버튼/VCP 배선의 공식 출처)
- RM0383 — STM32F411 레퍼런스 매뉴얼 (레지스터의 성경. W2에 I2C 할 때 열게 된다)
- ST 공식 "STM32 HAL 튜토리얼"류보다, 생성된 코드를 직접 읽는 §6이 더 빠르다
