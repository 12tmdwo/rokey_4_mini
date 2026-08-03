"""웹캠 + OAK-D가 YOLO로 잡은 dummy(map 좌표)를 PointCloud2로 발행 -> Nav2 코스트맵 장애물.

RPLIDAR가 dummy를 못 잡아서 로봇이 밀고 지나가는 걸 막는다. goal 결정은
goal_manager, 회피는 코스트맵 — 레이어가 달라 서로 간섭 없다.

웹캠(대략적 · 호모그래피 외삽 오차)이 먼저 위치를 잡고, 로봇이 가까워져 OAK-D가
같은 dummy를 반경 MATCH_RADIUS 안에서 잡으면 그 정밀 좌표로 갱신한다(잠금 · locked).
이후 웹캠은 무시한다 — car 추적(goal_manager_node)과 동일한 패턴. 웹캠이 dummy를
한 번도 못 본 경우에도 OAK-D가 먼저 잡으면 그걸 그대로 채택한다(fallback).

dummy는 고정이라 clearing 안 함(코스트맵 nav2.yaml에서 clearing:False). 웹캠이나
OAK-D가 잠깐 못 봐도 사라지면 안 되므로 마지막 좌표를 래치해서 계속 발행한다.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray

from mini_turtle4.detections import items, match

# ── 설정 ──────────────────────────────────────────────
WEBCAM_TOPIC = 'webcam/detections'
OAKD_TOPIC = 'oakd/detections'
CLOUD_TOPIC = 'dummy_cloud'
DUMMY_CLASS = 'dommy'       # 모델 학습 라벨 (오타 아님)
DUMMY_RADIUS = 0.1         # m — dummy 실제 반경. inflation 0.45m가 더 붙음
MATCH_RADIUS = 0.3          # m — OAK-D 보정 시 같은 dummy로 인정할 반경 (오검출 방지)
SPACING = 0.05              # m — map 해상도. 원판을 이 격자로 채운다
RATE = 2.0                  # Hz — 고정 장애물이라 낮게 계속 재발행(래치 유지)
Z = 0.2                     # m — voxel 높이 밴드(0~0.8) 안
FRAME = 'map'
# ──────────────────────────────────────────────────────


def disc_points(cx, cy, z, radius, spacing):
    """(cx,cy) 중심 반경 radius 원판을 spacing 격자점으로 채운다 -> [(x,y,z)].

    i,j 정수 격자로 도니까 i*i+j*j <= n*n 는 (i*spacing)^2+(j*spacing)^2 <=
    (n*spacing)^2 와 정확히 같다 (부동소수 오차 없음).
    """
    n = round(radius / spacing)
    return [(cx + i * spacing, cy + j * spacing, z)
            for i in range(-n, n + 1)
            for j in range(-n, n + 1)
            if i * i + j * j <= n * n]


class DummyObstacle(Node):

    def __init__(self):
        super().__init__('dummy_obstacle_node')
        self.marks = []         # 마지막으로 본 dummy 좌표 [(x,y)] — 래치
        self.locked = False     # OAK-D가 한 번이라도 확인했으면 True (이후 웹캠 무시)
        self.pub = self.create_publisher(PointCloud2, CLOUD_TOPIC, 10)
        self.create_subscription(Detection3DArray, WEBCAM_TOPIC, self.webcam_cb, 10)
        self.create_subscription(Detection3DArray, OAKD_TOPIC, self.oakd_cb, 10)
        self.create_timer(1.0 / RATE, self.tick)
        self.get_logger().info(
            f"{WEBCAM_TOPIC} + {OAKD_TOPIC}에서 '{DUMMY_CLASS}' -> {CLOUD_TOPIC} 발행")

    def webcam_cb(self, msg):
        if self.locked:
            return                  # OAK-D가 확인한 뒤로는 웹캠(대략적 좌표) 무시
        found = [(x, y) for cid, _, x, y in items(msg) if cid == DUMMY_CLASS]
        if found:               # 못 본 프레임은 이전 값 유지 (래치)
            self.marks = found

    def oakd_cb(self, msg):
        if not self.marks:
            # 웹캠이 아직 못 본 dummy를 OAK-D가 먼저 발견 -> 그대로 채택 (fallback)
            found = [(x, y) for cid, _, x, y in items(msg) if cid == DUMMY_CLASS]
            if found:
                self.marks = found
                self.locked = True
            return
        # 이미 아는 위치 각각을, 반경 안에서 같은 dummy로 보이면 정밀 좌표로 교체
        refined = [match(msg, x, y, DUMMY_CLASS, MATCH_RADIUS) or (x, y)
                   for x, y in self.marks]
        if refined != self.marks:
            self.marks = refined
            self.locked = True

    def tick(self):
        pts = [p for cx, cy in self.marks
               for p in disc_points(cx, cy, Z, DUMMY_RADIUS, SPACING)]
        h = Header()
        h.frame_id = FRAME
        h.stamp = self.get_clock().now().to_msg()
        self.pub.publish(create_cloud_xyz32(h, pts))


def main():
    rclpy.init()
    node = DummyObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


def _self_check():
    pts = disc_points(0.0, 0.0, 0.2, 0.15, 0.05)
    assert len(pts) == 29                                   # n=3 원판 격자점 수
    assert (0.0, 0.0, 0.2) in pts                           # 중심 포함
    assert all(x * x + y * y <= 0.15 ** 2 + 1e-9 for x, y, _ in pts)  # 반경 안
    assert all(z == 0.2 for _, _, z in pts)                 # 높이 일정
    assert len(disc_points(3.0, -2.0, 0.2, 0.15, 0.05)) == 29  # 이동해도 동일
    print('dummy_obstacle self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
