# mini_turtle4

맵 밖에 설치한 웹캠으로 **움직이는 차를 찾아** TurtleBot4가 계속 쫓아가고, 길에 놓인
**dummy는 피해서** 가는 ROS 2 패키지입니다. 로봇이 가까워지면 자신의 OAK-D로 차와
dummy를 직접 잡아 좌표를 정밀 보정하고, dummy 좌표는 Nav2 코스트맵에 장애물로 넣어
planner가 우회하게 합니다.

```
웹캠 YOLO ─ 호모그래피 ─→ 차 좌표(대략) ─┐
                                        ├─→ goal_manager ─→ Nav2 (추적)
OAK-D YOLO + depth ────→ 차 좌표(정밀) ─┘

웹캠 YOLO ─→ dummy 좌표(대략) ─┐
                              ├─→ dummy_obstacle ─→ 코스트맵 (회피)
OAK-D YOLO + depth ─→ dummy 좌표(정밀) ─┘
```

**차 추적**

1. 웹캠이 차를 찾아 map 좌표를 냅니다. 로봇에서 가장 가까운 **`car`**가 목표가 됩니다(dummy 제외).
2. 그 좌표 앞 0.5 m 지점으로 Nav2 goal을 보내며 접근합니다(계속 갱신).
3. 주행 중 OAK-D가 **같은 클래스**의 차를 반경 1 m 안에서 잡으면, 그때부터 OAK-D 좌표로
   계속 갱신 = **추적**. 이 순간부터 웹캠은 무시합니다. Nav2는 멈추지 않고 목표만
   바꾸며(선점), goal 재전송은 3 Hz로 상한을 둡니다.
4. OAK-D가 1.5 초 동안 차를 놓치면 goal을 취소하고 **제자리로 회전하며 재탐색**합니다.
   다시 잡으면 회전을 멈추고 추적을 재개합니다.

**dummy 회피**

- 웹캠이 `dommy`를 잡으면 그 대략적 좌표를 PointCloud2로 코스트맵에 넣어 장애물로
  만듭니다. RPLIDAR가 못 잡는 dummy를 planner가 우회합니다.
- 로봇이 가까워져 OAK-D가 **같은 dummy**를 반경 0.3 m 안에서 잡으면, 그 정밀 좌표로
  보정하고 잠금(locked) — 이후 웹캠은 무시합니다. 웹캠이 dummy를 한 번도 못 본
  경우에도 OAK-D가 먼저 잡으면 그걸 그대로 채택합니다(fallback). dummy는 고정으로
  가정하므로 한 번 확정되면 계속 그 좌표로 발행합니다.

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
git clone https://github.com/12tmdwo/rokey_4_mini.git
cd ~/turtlebot4_ws
colcon build --packages-select mini_turtle4
source install/setup.bash
```

패키지가 `src/rokey_4_mini/mini_turtle4/`에 놓이지만 colcon이 알아서 찾습니다.
대신 [`paths.py`](mini_turtle4/paths.py)의 `PKG`를 그 경로로 맞춰주세요.

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

localization(map_server·amcl)이 먼저 뜨고 **active된 뒤에야** Nav2(navigation)가 이어서
뜹니다 — 코스트맵이 맵(map_server)과 `map→odom` TF(amcl)를 필요로 하기 때문입니다.
`nav2_activate.sh`가 각 단계 lifecycle을 활성화하고 RViz도 함께 뜹니다. Nav2는
`config/nav2.yaml`(turtlebot4 기본 + dummy 장애물 소스)로 뜹니다.
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
| [`goal_manager_node.py`](mini_turtle4/goal_manager_node.py) | `TARGET_CLASS` | `car` — 추적할 클래스 (dummy 제외) |
| | `STOP_DIST` | `0.5` — 물체 앞 정지 거리 (m) |
| | `MATCH_RADIUS` | `1.0` — 같은 물체로 인정할 반경 |
| | `TRACK_RATE` | `3.0` — goal 재전송 상한 (Hz) |
| | `LOST_TIMEOUT` | `1.5` — 이만큼 놓치면 정지 (s) |
| [`dummy_obstacle_node.py`](mini_turtle4/dummy_obstacle_node.py) | `DUMMY_CLASS` | `dommy` — 장애물 클래스명 |
| | `DUMMY_RADIUS` | `0.1` — 마킹 반경 (m) |
| | `MATCH_RADIUS` | `0.3` — OAK-D 보정 시 같은 dummy로 인정할 반경 (m) |
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
| `oakd_locator_node` | OAK-D(compressed) YOLO + depth → TF → `oakd/detections` (주행 안 함) |
| `goal_manager_node` | 두 좌표로 차를 추적하는 Nav2 goal 결정·갱신·정지·탐색회전 |
| `dummy_obstacle_node` | `webcam/detections` + `oakd/detections`의 `dommy` → PointCloud2 `dummy_cloud` (코스트맵 장애물). OAK-D가 잡으면 정밀 좌표로 보정·잠금 |

좌표 계산과 ROS·주행은 파일이 분리돼 있습니다.

| 모듈 | 역할 | ROS 의존 |
|---|---|---|
| `homography.py` | 픽셀 → map 변환, 캘리브레이션 | ❌ |
| `depth_math.py` | RGB↔depth 픽셀 변환, deprojection, compressedDepth 디코드 | ❌ |
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
| `dummy_cloud` | `sensor_msgs/PointCloud2` | `map` |

클래스 이름은 `hypothesis.class_id`에 실립니다. 커스텀 메시지 패키지가 필요 없도록
표준 타입을 씁니다.

### 디버깅 도구

파이프라인에는 안 쓰이지만 확인용으로 남겨둔 것들입니다.

| 명령 | 용도 |
|---|---|
| `ros2 run mini_turtle4 depth_checker` | depth 화면 클릭 → 거리(m) 출력 |
| `ros2 run mini_turtle4 yolo_node` | OAK-D(compressed) 탐지 이미지 발행 + 물체 거리(cm) 로그 |
| `ros2 run mini_turtle4 cam_sub_node` | RGB 해상도·중앙 depth 로그 |
| `ros2 topic echo /robot4/dummy_cloud --field width` | 0보다 크면 dummy 마킹 중 |

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
- **RGB↔depth 픽셀 변환** — 단순 비율 변환입니다. 현재 파이프라인은 로봇에서 RGB와
  depth를 **같은 704×704**(HFOV 63.4° 동일)로 맞춰 발행해 픽셀이 1:1로 대응합니다.
  둘 다 **compressed**(`rgb/image_raw/compressed`, `stereo/.../compressedDepth`)로 받아
  WiFi 부담을 줄입니다. 카메라 설정을 바꾸면 `depth_checker`로 다시 확인하세요.
- **차 추적 방식** — goal을 3 Hz로 계속 재전송하며 쫓습니다(TRACK_RATE). 매 프레임 쏘면
  Nav2가 버벅여서 시간 상한을 뒀습니다. OAK-D가 차를 잡은 뒤로는 웹캠을 무시하고(locked),
  1.5 초 놓치면 `NavigateToPose`를 취소하고 `Spin` 액션(`behavior_server`)으로 제자리
  회전하며 재탐색합니다. 회전은 한 바퀴 다 돌아도 못 찾으면 스스로 다시 돌고
  (`nav_controller.Navigator.spin_search`), OAK-D가 재획득하면 그제서야 멈추고 추적을
  재개합니다.
- **dummy를 코스트맵에 넣는 이유** — RPLIDAR가 못 잡는 장애물이라 YOLO 좌표를
  PointCloud2로 obstacle 관측원에 흘려 넣습니다. 고정 장애물이라 `clearing:False`로 그
  세션 동안 유지합니다(디스크 저장 아님 — 재시작하면 웹캠이 다시 봐서 재생성). 설정은
  [`config/nav2.yaml`](config/nav2.yaml)에 turtlebot4 기본 + `dummy` 소스로 들어 있습니다.
- **dummy도 OAK-D로 보정하는 이유** — 웹캠은 맵 밖에서 비스듬히 찍고 undistort도 안
  해서, 화면 가장자리·호모그래피 외삽 구간은 오차가 코스트맵 inflation 마진(0.45 m)을
  넘을 수 있습니다. `clearing:False`라 한 번 잘못 박히면 세션 내내 그대로라 초기 오차가
  누적되면 위험합니다. 그래서 로봇이 가까워져 OAK-D가 같은 dummy를 잡으면(반경 0.3 m)
  그 정밀 좌표로 덮어쓰고 잠급니다 — 이후 웹캠 갱신은 무시합니다(차 추적과 동일 패턴).
  웹캠이 dummy를 아예 못 본 경우에도 OAK-D가 먼저 잡으면 그대로 채택해, 웹캠 단일 관측
  실패가 회피 기능 전체를 무력화하지 않게 합니다.

## 라이선스

Apache-2.0
