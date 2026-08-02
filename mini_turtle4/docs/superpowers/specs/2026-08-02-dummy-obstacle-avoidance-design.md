# dummy 장애물 회피 설계 (YOLO → Nav2 costmap)

날짜: 2026-08-02

## 문제

`dommy`(장애물 프롭)는 RPLIDAR 평면에 안 걸려서 코스트맵에 안 잡히고, 그래서
로봇이 그냥 밀고 지나간다. dummy는 이미 YOLO 클래스로 인식되고 두 로케이터가
map 좌표까지 뽑아 발행 중이므로, 그 좌표를 코스트맵에 장애물로 넣어 planner가
우회하게 만든다.

## 결정 사항 (확정)

- **dummy는 고정** (주행 중 안 움직임) → clearing 불필요, 같은 자리에 계속 마킹.
- **감지 소스는 웹캠만** (`webcam/detections`). 웹캠이 맵 전체를 봐서 멀리서부터
  마킹 → global planner가 처음부터 우회. (로봇캠 추가는 나중에 구독 한 줄이면 됨)
- **클래스명 `dommy`** (모델 학습 라벨 그대로).
- **마킹 반경 0.15m** (지름 0.3m 물체). inflation 0.45m가 더 붙어 실질 회피 반경 약 0.6m.

## 접근법 선택

| 방법 | 결정 |
|---|---|
| **A. PointCloud2 → costmap 관측원** | **채택.** Nav2 정공법. 노드 1개 + 설정 몇 줄. |
| B. keepout 필터 (마스크 이미지) | 탈락 — 마스크를 미리 그려야 해 실시간 YOLO 좌표를 못 씀. |
| C. C++ costmap 레이어 플러그인 | 탈락 — 고정 장애물엔 과함(빌드·pluginlib 등록). |

## 구조

```
webcam_locator_node ──webcam/detections──> dummy_obstacle_node ──dummy_cloud──> Nav2 costmap
   (YOLO+호모그래피)     (Detection3DArray)    (dommy만 필터·래치)   (PointCloud2)   (obstacle/voxel layer)
```

- goal 결정(goal_manager)과 회피(costmap)는 **레이어가 분리**돼 서로 간섭 없음.
- dummy 마킹 → inflation → global planner 우회, local controller 근접 회피.

## 컴포넌트

### 1. `mini_turtle4/dummy_obstacle_node.py` (신규)
- 구독: `webcam/detections` (Detection3DArray, map 프레임)
- `dommy` 클래스만 필터 → 좌표 리스트.
- **래치**: dummy가 있는 프레임이 오면 최신 좌표로 교체, 없는 프레임은 이전 값 유지
  (고정 장애물이라 웹캠이 잠깐 못 봐도 코스트맵에서 사라지면 안 됨).
- 2Hz 타이머로 `dummy_cloud`(PointCloud2, **frame_id=map**, z=0.2) 발행.
- 각 dummy 좌표 주위에 반경 `DUMMY_RADIUS`(=0.15) 원판 점 뭉치를 map 해상도(0.05) 간격으로 생성.
- PointCloud2 조립은 `sensor_msgs_py.point_cloud2.create_cloud_xyz32` 사용 (기존 의존성).
- 튜닝 상수: `DUMMY_CLASS='dommy'`, `DUMMY_RADIUS=0.15`, `RATE=2.0`, `Z=0.2`.

### 2. `mini_turtle4/config/nav2.yaml` (신규 — turtlebot4 기본 복사 + 2곳 수정)
turtlebot4_navigation 기본 nav2.yaml을 복사하고 **관측원만 추가**한다.
파일 상단에 "turtlebot4 기본 복사본, dummy 소스만 추가, 업뎃 시 재동기화" 주석.

- **global_costmap → obstacle_layer**
  ```yaml
  observation_sources: scan dummy
  dummy:
    topic: dummy_cloud
    data_type: "PointCloud2"
    marking: True
    clearing: False
    obstacle_max_range: 100.0   # map 원점 기준 거리라 기본 2.5면 멀리 있는 dummy가 잘림
    max_obstacle_height: 2.0
    min_obstacle_height: 0.0
  ```
- **local_costmap → voxel_layer**: 동일한 `dummy` 블록 추가 (`observation_sources: scan dummy`).
  local은 rolling window지만 obstacle_max_range도 map 원점 기준이라 동일하게 크게 둔다.

### 3. 런치 수정
- `robot_bringup.launch.py`: nav2 include에 `('params_file', <config/nav2.yaml>)` 전달.
  경로는 `get_package_share_directory('mini_turtle4')/config/nav2.yaml`.
- `bringup.launch.py`: `dummy_obstacle_node` Node 추가 (namespace robot4, TF remap 동일).

### 4. `setup.py`
- `console_scripts`에 `dummy_obstacle_node` 엔트리포인트 등록.
- `data_files`에 `config/*.yaml` → `share/mini_turtle4/config/` 설치.

## 데이터 흐름 / 프레임

- 좌표는 전부 map 프레임(z=0 평면). PointCloud2도 map 프레임, z=0.2로 올려
  costmap 높이 밴드(voxel origin_z 0.0 ~ 0.8) 안에 들어가게 함.
- `clearing:False` → 마킹된 셀은 코스트맵 리셋 전까지 유지. 고정 dummy에 정확.

## 에러 / 엣지 케이스

- **웹캠이 dummy를 아직 못 봄**: 빈 cloud 발행(또는 발행 안 함) → 코스트맵 변화 없음. 정상.
- **dummy를 실제로 치웠을 때**: `clearing:False`라 코스트맵엔 남음. 미니프로젝트 한 판엔 무방.
  필요해지면 "N초 미검출 시 만료" 로직 추가 (지금은 YAGNI).
- **dummy 여러 개**: 한 프레임에 같이 보이면 전부 마킹됨. 개별로 나타났다 사라지면
  프레임 단위 래치라 놓칠 수 있음 — 단일/동시 감지 케이스만 보장. 필요 시 per-dummy 추적.
- **obstacle_max_range**: map 원점 기준 거리 계산이라 반드시 크게(100). 안 그러면
  원점에서 먼 dummy가 소스 단계에서 잘려 마킹 안 됨.

## 테스트

- `dummy_obstacle_node`의 원판 점 생성 함수에 `_self_check`(assert): 중심/반경/점 개수 검증.
- 통합 확인: 웹캠 앞에 dummy → RViz 코스트맵에 마킹 블롭 + inflation 확인 → goal 주면 우회하는지.

## 커플링 / 유지보수

- nav2.yaml 통째 복사를 떠안음 (Nav2가 부분 오버레이 미지원). turtlebot4 업데이트 시
  수동 재동기화. 파일 상단 주석으로 명시.
