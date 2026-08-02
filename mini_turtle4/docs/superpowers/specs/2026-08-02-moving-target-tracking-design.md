# 움직이는 차량 추적 (moving-target tracking)

작성일: 2026-08-02
대상 파일: `mini_turtle4/goal_manager_node.py` (한 파일만 수정, 새 파일 없음)

## 목표

정지한 물체 앞에 한 번 다가가 서는 기존 동작을, **움직이는 차량을 계속 쫓다가
놓치면 멈추는** 동작으로 바꾼다. 차량은 하나뿐이라 객체 혼동은 고려하지 않는다.

## 동작 (상태 기계)

```
[탐색]  웹캠이 차를 보고 → 로봇이 그쪽으로 접근 (OAK-D는 아직 못 봄)
   │      웹캠 좌표로 goal을 갱신하며 접근 (스로틀 적용)
   ▼  OAK-D가 차를 처음 잡는 순간
[추적]  OAK-D 좌표로 goal을 매번 갱신 = 움직이는 차를 쫓음
   │      이 순간부터 웹캠은 완전 무시 (기존 locked 유지)
   ▼  OAK-D가 LOST_TIMEOUT 동안 연속으로 차를 놓침
[정지]  nav.cancel() → 로봇 정지
   │      OAK-D가 다시 잡으면 → [추적]으로 복귀 (재개)
```

- **탐색 → 추적**: OAK-D가 현재 목표와 같은 물체를 잡으면 전환. 기존
  `oakd_cb` + `locked` 로직 그대로.
- **추적 중 웹캠**: 완전 무시. 기존과 동일.
- **놓침 판정**: OAK-D가 `LOST_TIMEOUT`(기본 1.5s) 동안 한 번도 목표를 못 잡으면
  놓친 것으로 본다. 한 프레임 미검출은 무시 = YOLO 노이즈 디바운스.
- **정지 후 재개**: OAK-D가 다시 잡으면 추적 재개. 짧은 가림/노이즈에 강함.
  (선택지 중 "재개" 채택)

## 기존과 달라지는 점 (딱 두 가지)

1. `UPDATE_THRESH`(15cm 거리 게이트) 제거 → 매 검출마다 goal 갱신 = 실제 추적.
   무한 재전송을 막기 위해 **거리 게이트 대신 시간 스로틀**(`TRACK_RATE`)로 교체.
2. OAK-D가 놓쳤을 때 현재는 "마지막 goal 유지"(`refine`가 False 반환)인데,
   → `LOST_TIMEOUT` 초과 시 `nav.cancel()`로 **정지**.

## 방식 선택: 액션 재전송 (B안)

기존 `Navigator`(NavigateToPose 액션) 구조를 그대로 쓰고, `nav.go()`를 매 검출마다
다시 부르되 `TRACK_RATE`로 상한을 건다. 놓치면 `nav.cancel()`.

- **채택 이유**: "놓치면 정지"가 어차피 goal 취소를 요구해서, follow_point.xml의
  `KeepRunningUntilFailure`("무한 추종") 장점을 절반밖에 못 쓴다. 열린 공간
  추격 중엔 recovery 진입이 드물어 preempt 문제도 3Hz 스로틀이면 거의 안 나타난다.
  새 파일·data_files·behavior_tree 필드·goal_update 발행이 전부 불필요.
- **버리는 대안(A안)**: follow_point.xml + `/robot4/goal_update` 토픽. Nav2 내부
  goal 재수락이 없어 더 매끄럽지만 구성 요소가 많다. **실제 로봇에서 B안의 goal
  재수락이 눈에 띄게 버벅일 때 올려붙이는 업그레이드 카드로 남긴다.**

## 튜닝 상수 (하드웨어라 현장 조정 필요)

| 상수 | 기본값 | 의미 |
|---|---|---|
| `TRACK_RATE` | 3.0 Hz | 추적 중 goal 재전송 상한 |
| `LOST_TIMEOUT` | 1.5 s | 이 시간 연속 미검출이면 정지 (디바운스) |
| `STOP_DIST` | 0.5 m | 물체 앞 정지 거리 (기존 유지) |

## 검증

- `nav_controller.py`의 기존 `_self_check`(approach_point) 유지.
- goal_manager는 ROS·TF 의존이라 단위 테스트가 어렵다. 놓침 판정의 순수 로직
  (마지막 검출 시각 대비 경과가 `LOST_TIMEOUT` 초과 → 정지)은 시각을 인자로 받는
  작은 순수 함수로 빼서 `assert` self-check 한 줄을 남긴다.
- 실로봇 확인: 차를 움직여 로봇이 따라오는지, 차를 화면 밖으로 빼면
  ~1.5s 뒤 멈추는지, 다시 넣으면 재개하는지.

## 미해결 (이 스펙 범위 밖)

- `cam_sub_node`/`yolo_node` 정리 여부 — 별개 작업.
- follow_point.xml(A안) 전환 — B안이 실측에서 버벅일 때만.
</content>
