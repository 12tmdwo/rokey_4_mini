# mini_turtle4

맵 밖에 설치한 웹캠으로 물건을 찾아 TurtleBot4를 그 앞까지 보내는 ROS 2 패키지입니다.
로봇이 가까워지면 자신의 OAK-D로 같은 물건을 다시 재서 목표를 더 정확하게 갱신합니다.

```
웹캠 YOLO ─ 호모그래피 ─→ map 좌표 (대략)  ─┐
                                            ├─→ goal_manager ─→ Nav2
OAK-D YOLO + depth ────→ map 좌표 (정밀)  ─┘
```

1. 웹캠이 물건을 찾아 map 좌표를 냅니다. 로봇에서 가장 가까운 물건이 목표가 됩니다.
2. 그 좌표 앞 0.5 m 지점으로 Nav2 goal을 보냅니다.
3. 주행 중 OAK-D가 **같은 클래스**의 물건을 반경 1 m 안에서 잡으면, 더 정확한 좌표로
   goal을 갱신합니다. Nav2는 멈추지 않고 목표만 바꿉니다(선점).
4. 너무 가까워져 depth가 무효가 되거나 화면 밖으로 나가면 마지막 goal을 유지합니다.

> ⚠️ **이 저장소는 특정 환경(사용자 `rokey`, 네임스페이스 `robot4`)에 맞춰져 있습니다.**
> 다른 곳에서 쓰시려면 아래 [고쳐야 할 값](#고쳐야-할-값)을 먼저 수정하세요.

---

## 요구사항

- Ubuntu 22.04 / ROS 2 Humble
- TurtleBot4 (OAK-D 포함), Nav2, `turtlebot4_navigation`, `turtlebot4_viz`
- Python: `ultralytics`, `opencv-python`, `numpy`, `pyyaml`
- 맵 밖에 비스듬히 설치한 USB 웹캠

**OAK-D는 RGBD 파이프라인이어야 합니다.** 기본값이 RGB면 depth가 안 나와서
`oakd_locator_node`가 동작하지 않습니다.

```bash
ros2 param get /robot4/oakd camera.i_pipeline_type   # RGBD 여야 함
```

## 데이터 파일

| 파일 | 저장소 포함 | 비고 |
|---|---|---|
| `resource/my_map.yaml` + `.pgm` | ✅ | `turtlebot4_navigation/slam.launch.py`로 SLAM 후 저장 |
| `resource/homography.npy` | ✅ | 아래 [3단계](#3-호모그래피-캘리브레이션)에서 생성 |
| `~/turtlebot4_ws/best.pt` | ❌ | YOLO 학습 결과 (22 MB). 없으면 `yolov8n.pt`로 대체 가능 |

> ⚠️ 들어 있는 맵과 호모그래피는 **저희 현장 전용**입니다. 다른 곳·다른 웹캠 위치에서는
> **에러 없이 좌표만 틀립니다.** 환경이 다르면 아래 준비 단계로 둘 다 새로 만드세요.

경로는 [`mini_turtle4/paths.py`](mini_turtle4/paths.py) 한 곳에 모여 있습니다.
저장소를 다른 데 받으셨으면 `PKG` 한 줄만 고치면 됩니다.

---

## 설치

```bash
cd ~/turtlebot4_ws/src
git clone <이 저장소> mini_turtle4
cd ~/turtlebot4_ws
colcon build --packages-select mini_turtle4
source install/setup.bash
```

## 준비 (최초 1회)

### 1. 맵 만들기

SLAM으로 맵을 만들고 `resource/my_map.yaml`(+ `.pgm`)로 저장합니다. 두 파일은 반드시
같은 폴더에 두세요 — yaml 안의 `image:`가 상대 경로입니다.

### 2. 바닥 기준점 4개 정하기

호모그래피는 **바닥 평면** 기준이라, 맵과 실제 현장 양쪽에서 식별 가능한 지점이
필요합니다. **벽 모서리**가 가장 좋습니다.

```bash
cd ~/turtlebot4_ws/src/mini_turtle4
python3 -m mini_turtle4.map_point_picker
```

맵 이미지가 뜹니다. 기준점 4개를 클릭하면 `MAP_POINTS`에 붙여넣을 형태로 출력됩니다.
(`r` 초기화, `q` 종료)

- **웹캠 화면에 4점이 모두 보여야** 합니다
- **넓게 분산**시키세요. 좁게 모이면 오차가 크게 증폭됩니다

출력된 좌표를 [`mini_turtle4/homography.py`](mini_turtle4/homography.py)의 `MAP_POINTS`에
넣습니다.

### 3. 호모그래피 캘리브레이션

```bash
cd ~/turtlebot4_ws/src/mini_turtle4
python3 -m mini_turtle4.homography
```

웹캠 화면에서 **`MAP_POINTS`와 같은 순서로** 4점을 클릭합니다. 벽 모서리를 쓰신다면
반드시 **벽이 바닥에 닿는 밑동**을 클릭하세요 — 윗부분을 찍으면 벽 높이만큼 전부
어긋납니다.

`homography.npy`가 저장되고 점별 오차가 cm 단위로 출력됩니다.
**오차가 10 cm를 넘으면 `r`로 초기화하고 다시 하세요.**

> 캘리브레이션 후 **웹캠을 움직이면 `homography.npy`는 무효**입니다. 다시 하세요.

---

## 실행

터미널 2개면 됩니다.

### 터미널 A — Nav2 + RViz

```bash
cd ~/turtlebot4_ws && source install/setup.bash
ros2 launch mini_turtle4 robot_bringup.launch.py
```

localization, Nav2, RViz가 함께 뜨고 `nav2_activate.sh`가 lifecycle을 활성화합니다.
`전부 active`가 나오면 준비된 것입니다.

**RViz에서 `2D Pose Estimate`로 초기 위치를 찍으세요.** AMCL은 위치를 기억하지 않으므로
재시작할 때마다 매번 필요합니다.

### 터미널 B — 파이프라인

```bash
cd ~/turtlebot4_ws && source install/setup.bash
ros2 launch mini_turtle4 bringup.launch.py namespace:=robot4
```

코드를 고칠 땐 이 터미널만 `colcon build` 후 재시작하면 됩니다. 초기 위치는 안 날아갑니다.

### 물건 놓는 위치

**캘리브레이션한 4점 사각형 안**에 두세요. 밖은 호모그래피 외삽이라 오차가 급격히 커집니다.
렌즈 왜곡 보정(undistort)을 하지 않으므로 **화면 가장자리도 피하세요.**

---

## 고쳐야 할 값

| 파일 | 상수 | 현재 값 |
|---|---|---|
| [`paths.py`](mini_turtle4/paths.py) | `PKG` | `/home/rokey/turtlebot4_ws/src/mini_turtle4` — 저장소 위치 |
| | `MODEL` | `/home/rokey/turtlebot4_ws/best.pt` |
| [`homography.py`](mini_turtle4/homography.py) | `MAP_POINTS` | 현장 실측 4점 — **반드시 교체** |
| | `CAM_INDEX` | `2` (`ls /dev/video*`로 확인) |
| [`webcam_locator_node.py`](mini_turtle4/webcam_locator_node.py) | `CONF`, `RATE`, `SHOW` | `0.6`, `5.0 Hz`, `True` |
| [`goal_manager_node.py`](mini_turtle4/goal_manager_node.py) | `STOP_DIST` | `0.5` — 물체 앞 정지 거리 (m) |
| | `UPDATE_THRESH` | `0.15` — 이만큼 움직여야 goal 갱신 |
| | `MATCH_RADIUS` | `1.0` — 같은 물체로 인정할 반경 |
| [`robot_bringup.launch.py`](launch/robot_bringup.launch.py) | `NAMESPACE` | `/robot4` |
| [`depth_checker.py`](mini_turtle4/depth_checker.py) | 토픽 2개 | `/robot4/...` |

파일 경로는 전부 `paths.py` 하나에서 나옵니다. 맵·호모그래피는 `PKG` 기준이라
저장소를 옮겼을 때 **한 줄만** 고치면 됩니다.

`CAM_INDEX`도 `homography.py`에만 있고 `webcam_locator_node.py`가 import합니다.
캘리브레이션과 실행이 다른 카메라를 열면 안 되기 때문입니다.

> `paths.py`는 `install/`이 아니라 **소스 트리**를 가리킵니다. 캘리브레이션으로 새로
> 만든 `homography.npy`를 노드가 바로 읽게 하려는 것입니다. `share/`를 쓰면 매번
> `colcon build`를 해야 하고, 빼먹으면 **에러 없이 옛 좌표를 계속 씁니다.**

---

## 구성

### 노드

| 이름 | 역할 |
|---|---|
| `webcam_locator_node` | 웹캠 YOLO → bbox 하단 중앙 → 호모그래피 → `webcam/detections` |
| `oakd_locator_node` | OAK-D YOLO + depth → TF → `oakd/detections` (주행 안 함) |
| `goal_manager_node` | 두 좌표를 받아 Nav2 goal 결정·갱신 |

좌표 계산과 ROS·주행은 파일이 분리돼 있습니다.

| 모듈 | 역할 | ROS 의존 |
|---|---|---|
| `homography.py` | 픽셀 → map 변환, 캘리브레이션 | ❌ |
| `depth_math.py` | RGB↔depth 픽셀 변환, deprojection | ❌ |
| `detections.py` | `Detection3DArray` 조립·매칭 | 메시지만 |
| `nav_controller.py` | 접근점 계산, Nav2 액션 래퍼 | ✅ |

각 모듈은 자체 검사를 갖고 있습니다.

```bash
cd ~/turtlebot4_ws/src/mini_turtle4
python3 -m mini_turtle4.homography --test
python3 -m mini_turtle4.depth_math --test
python3 -m mini_turtle4.detections --test
python3 -m mini_turtle4.nav_controller --test
python3 -m mini_turtle4.map_point_picker --test
```

### 토픽

| 토픽 | 타입 | 프레임 |
|---|---|---|
| `webcam/detections` | `vision_msgs/Detection3DArray` | `map` |
| `oakd/detections` | `vision_msgs/Detection3DArray` | `map` |

클래스 이름은 `hypothesis.class_id`에 실립니다. 커스텀 메시지 패키지가 필요 없도록
표준 타입을 씁니다.

### 디버깅 도구

파이프라인에는 안 쓰이지만 확인용으로 남겨둔 것들입니다.

| 명령 | 용도 |
|---|---|
| `ros2 run mini_turtle4 depth_checker` | depth 화면 클릭 → 거리(m) 출력 |
| `ros2 run mini_turtle4 yolo_node` | OAK-D 탐지 결과를 이미지로 발행 |
| `ros2 run mini_turtle4 cam_sub_node` | RGB 해상도·중앙 depth 로그 |

---

## 문제 해결

### `navigate_to_pose 서버 아직 없음`

Nav2 lifecycle이 `unconfigured`나 `inactive`에 멈춘 것입니다. WiFi + Discovery Server
환경에서는 lifecycle 전환 서비스 응답이 자주 타임아웃 납니다. **코드 문제가 아닙니다.**

```bash
./src/mini_turtle4/scripts/nav2_activate.sh robot4
```

멈춘 노드만 골라 밀어 올립니다. 한 번에 안 되면 다시 실행하세요.

### `Invalid frame ID "map" ... frame does not exist`

TF를 **듣는 쪽**의 문제일 수 있습니다. `tf2_ros`의 `TransformListener`는
네임스페이스를 무시하고 절대 경로 `/tf`를 구독합니다
(`transform_listener.py:85`). 네임스페이스를 쓰면 리맵이 필수입니다.

```python
remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
```

`bringup.launch.py`에는 이미 적용돼 있습니다. 직접 확인하실 땐:

```bash
ros2 run tf2_ros tf2_echo map base_link --ros-args \
    -r /tf:=/robot4/tf -r /tf_static:=/robot4/tf_static
```

프레임 이름은 `robot4/base_link`가 아니라 접두사 없는 **`base_link`** 입니다.

### 한 물건에 라벨이 두 개 붙음

```
car     px(126, 407) -> map(2.43, 1.58)
dommy   px(126, 407) -> map(2.43, 1.58)
```

YOLO의 NMS는 클래스별로 동작해서 박스가 겹쳐도 클래스가 다르면 둘 다 남습니다.
`goal_manager`는 **클래스 이름 일치**로 목표를 추적하므로, 라벨이 흔들리면 OAK-D 보정이
조용히 실패합니다. 두 로케이터 모두 `agnostic_nms=True`로 막아뒀습니다.

### 노드가 갑자기 전부 사라짐

launch 프로세스가 죽으면 그 자식 노드가 전부 따라 죽습니다. VS Code 통합 터미널은
창 새로고침이나 패널 정리에도 함께 종료되니, **별도 터미널 앱**을 쓰세요.

```bash
setsid nohup ros2 launch mini_turtle4 robot_bringup.launch.py > ~/nav2.log 2>&1 &
```

### depth가 안 나옴

`/robot4/oakd/stereo/...`가 없으면 카메라가 RGB 전용 파이프라인입니다. 로봇에서 OAK-D를
RGBD로 다시 띄우세요.

---

## 설계 노트

- **bbox 하단 중앙을 쓰는 이유** — 호모그래피는 바닥 평면 변환이라, bbox 중심을 쓰면
  물건 높이만큼 좌표가 밀립니다. 물건이 바닥에 닿는 지점이어야 합니다.
- **undistort를 안 하는 이유** — 체커보드 캘리브레이션을 하지 않았습니다. 렌즈 왜곡은
  화면 가장자리에서 커지므로, 물건을 화면 중앙 쪽에 두는 것으로 대응합니다.
- **RGB↔depth 픽셀 변환** — 단순 비율 변환입니다. 실측 확인 결과 RGB 320×320과
  depth 704×704가 같은 FOV의 정사각형이고 depth가 RGB 프레임에 정렬돼 있어,
  RGB 중심 (160,160) → (352,352)이 depth 주점 (353.4, 354.1)과 1 px 차이였습니다.
  카메라 설정을 바꾸면 `depth_checker`로 다시 확인하세요.
- **goal 갱신 임계값** — 물체 좌표가 15 cm 넘게 움직여야 갱신합니다. 로봇 위치가 아니라
  **물체 위치**를 기준으로 비교하므로, 로봇이 곡선 주행해도 불필요한 갱신이 안 생깁니다.

## 라이선스

Apache-2.0
