# 실시간 스케줄링 이론 — 축 A(현대차)의 이론적 바탕

> 학습 자료(축 A · 중요). [freertos-concepts.md](freertos-concepts.md)가 "FreeRTOS는 이렇게 동작한다"
> 라면, 이 문서는 "그래서 마감시한을 지키는지 어떻게 따지는가"의 이론이다.
> **Pintos(OS 커널 스케줄러/우선순위 기부)를 해봤다면 이 문서는 복습 + 실물 적용이다.**

---

## 1. WCET — 평균이 아니라 최악을 설계한다

- **WCET(Worst-Case Execution Time)**: 어떤 코드 블록이 걸릴 수 있는 **최대** 시간.
- 실시간 시스템의 스케줄 가능성(schedulability)은 평균이 아니라 **WCET로 따진다.** "평균은
  여유 있는데 최악에 마감시한을 놓친다"가 실패다.
- 이 프로젝트에서 추론 태스크의 WCET = [measurement-methodology.md](measurement-methodology.md)로 잰
  추론 사이클의 **p99/max**. 즉 측정이 이론의 입력이 된다.

---

## 2. 고정 우선순위 선점형 — 복습과 심화

FreeRTOS 규칙: **Ready 태스크 중 최고 우선순위가 항상 CPU를 갖고, 더 높은 게 Ready가 되면 즉시 선점.**

- **Rate-Monotonic(RM)**: "주기가 짧은 태스크에 높은 우선순위"를 주는 고전적 배정 원칙.
  100Hz 센서 > 10Hz 추론처럼. 최적(optimal) 고정우선순위 배정으로 알려짐.
- **스케줄 가능성 경계(Liu & Layland)**: n개 주기 태스크가 각 이용률 Uᵢ=Cᵢ/Tᵢ일 때,
  ΣUᵢ ≤ n(2^(1/n) − 1) 이면 RM으로 항상 스케줄 가능(충분조건). n→∞에서 약 0.693.
  → "CPU를 69%까지만 쓰면 안전"이라는 감. (필요충분은 아래 §4 응답시간 분석.)

> CS 연결: 이건 님이 OS 수업에서 봤을 수 있는 **실시간 스케줄링** 그 내용이다. Pintos의
> priority scheduling을 "주기·마감시한" 관점으로 다시 보는 것.

---

## 3. 우선순위 역전 & 상속 — Pintos donation의 실물

- **우선순위 역전(priority inversion)**: 낮은 태스크 L이 공유 자원(뮤텍스)을 쥔 채,
  높은 태스크 H가 그 자원을 기다리는데, 중간 태스크 M이 L을 선점 → H가 M 때문에 무한정 밀림.
- **우선순위 상속(priority inheritance)**: 뮤텍스가 L의 우선순위를 잠깐 H로 끌어올려 M의 선점을 막음.
  → **Pintos의 priority donation 과제가 정확히 이것.** FreeRTOS 뮤텍스엔 이게 내장, 세마포어엔 없음.
- 이 프로젝트에서: UART 로깅 뮤텍스를 액션 태스크와 로거가 공유하면 지터에 역전이 나타날 수 있다.
  **면접에서 "지터의 원인을 우선순위 역전으로 분석하고 상속으로 해결"은 강한 서사.**

---

## 4. 응답시간 분석(RTA) — config 스윕이 곧 실험적 RTA

태스크의 **응답시간 R** = 자기 실행시간 C + 자기보다 높은 태스크들의 간섭(interference):

```
R_i = C_i + Σ_(j: 우선순위 높음) ⌈R_i / T_j⌉ · C_j
```

- 직관: 내가 도는 동안 더 높은 태스크가 몇 번 끼어들어 나를 밀어내는가.
- **R_i ≤ D_i(마감시한)** 이면 그 태스크는 스케줄 가능.
- 이 프로젝트의 축 A 실험은 이 식을 **이론으로 풀지 않고 측정으로 관측**하는 것:
  추론 우선순위를 액션보다 높/낮게 바꾸면, 액션의 간섭항이 커지고/사라지고 → 액션 지연 p99가 변한다.

---

## 5. 선점 지점 만들기 — 청킹(chunking)

긴 추론이 **선점 불가능한 한 덩어리**면, 그게 도는 동안 도착한 마감시한 이벤트는 추론이 끝날
때까지 기다린다(= 큰 blocking). 대응:

- **추론을 낮은 우선순위로**: 액션이 추론을 선점 → 액션 지연 보존(가장 단순).
- **청킹**: 추론을 레이어/블록 단위로 쪼개고 사이에 선점 지점(yield 또는 짧은 블로킹)을 둠 →
  추론이 높은 우선순위여도 마감시한 이벤트가 끼어들 틈이 생김. 대신 추론 총시간은 늘어남.
- **트레이드오프**: 결정성(마감시한 준수) ↔ 처리량(추론 총 시간). 이걸 **측정으로 증명**하는 게 축 A 그래프.

---

## 6. 지터(jitter)의 종류

- **릴리스 지터(release jitter)**: 태스크가 "깨어나야 할 시각"과 실제 Ready 시각의 차.
- **응답 지터(response jitter)**: 매 주기 응답시간의 흔들림(p50 대비 p99의 폭).
- 주기 작업은 `vTaskDelayUntil`(절대주기)로 드리프트를 없애고, 그 위에 남는 지터가 **스케줄링에서 온 것**임을 분리해야 측정이 깨끗하다([freertos-concepts.md](freertos-concepts.md) §3).

---

## 7. Pintos ↔ FreeRTOS 매핑 (README에 쓸 대비표)

| 개념 | Pintos | FreeRTOS |
|---|---|---|
| 스케줄러 | priority scheduler (과제로 구현) | 고정 우선순위 선점형(내장) |
| 우선순위 역전 해결 | priority donation (구현 과제) | 우선순위 상속(뮤텍스 내장) |
| 동기화 | semaphore / lock / condvar | 세마포어 / 뮤텍스 / 큐 |
| 생산자-소비자 | bounded buffer | 큐(값 복사 + 블로킹) |
| 실행 단위 | 커널 스레드, 스택별 | 태스크, 고정 스택 |
| 메모리 | 페이지/가상메모리 있음 | **MMU 없음, 물리주소 공유** |

> 이 표 하나가 "OS 이론을 실물 실시간 제약에서 검증했다"는 서사의 뼈대다.

---

## 8. 자기 점검

1. 왜 스케줄 가능성을 평균이 아니라 WCET로 따지나?
2. Rate-Monotonic은 우선순위를 무엇 기준으로 배정하나?
3. Pintos의 priority donation은 FreeRTOS의 무엇에 해당하나? 세마포어엔 왜 없나?
4. 추론을 청킹하면 무엇이 좋아지고 무엇이 나빠지나?
5. 응답시간 식에서 ⌈R/T_j⌉·C_j 항은 물리적으로 무엇을 뜻하나?

---

### 더 읽을 것
- Liu & Layland (1973), *Scheduling Algorithms for Multiprogramming in a Hard-Real-Time Environment* — RM의 원전
- Buttazzo, *Hard Real-Time Computing Systems* — RTA/RM 교과서 (해당 장만)
- freertos.org "Kernel > RTOS Fundamentals"