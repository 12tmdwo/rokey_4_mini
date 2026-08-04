"""웹캠이 YOLO로 잡은 dummy(map 좌표)를 PointCloud2로 발행 -> Nav2 local 코스트맵 장애물.

RPLIDAR가 dummy를 못 잡아서 로봇이 밀고 지나가는 걸 막는다. goal 결정은 goal_manager,
회피는 코스트맵이라 서로 간섭 없다. dummy는 local_costmap에만 넣는다(global은 잔상이
남아 빼기로 함 — nav2.yaml plugins 참고).

웹캠(호모그래피 좌표)만 쓴다. OAK-D는 시야가 좁아 dummy 소스로 안 쓴다. dummy는 고정이라
clearing 안 함(clearing:False). 잠깐 못 봐도 사라지면 안 되므로 마지막 좌표를 래치해서
계속 발행한다.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray

from mini_turtle4.detections import items

# ── 설정 ──────────────────────────────────────────────
WEBCAM_TOPIC = 'webcam/detections'
CLOUD_TOPIC = 'dummy_cloud'
DUMMY_CLASS = 'dommy'       # 모델 학습 라벨 (오타 아님)
DUMMY_RADIUS = 0.1         # m — dummy 실제 반경. inflation 0.45m가 더 붙음
SPACING = 0.05              # m — map 해상도. 원판을 이 격자로 채운다
RATE = 2.0                  # Hz — 고정 장애물이라 낮게 계속 재발행(래치 유지)
Z = 0.2                     # m — voxel 높이 밴드(0~0.8) 안
FRAME = 'map'
LOST_TIMEOUT = 5.0          # s — 웹캠이 이만큼 dummy 못 보면 치운 걸로 보고 마크 제거
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


def expired(last_seen, now, timeout):
    """last_seen 이후 timeout초 넘게 갱신 없으면 True (아직 못 봤으면 False)."""
    return last_seen is not None and now - last_seen > timeout


class DummyObstacle(Node):

    def __init__(self):
        super().__init__('dummy_obstacle_node')
        self.marks = []         # 마지막으로 본 dummy 좌표 [(x,y)] — 래치
        self.last_seen = None   # 웹캠이 마지막으로 dummy를 본 시각(초)
        self.pub = self.create_publisher(PointCloud2, CLOUD_TOPIC, 10)
        self.create_subscription(Detection3DArray, WEBCAM_TOPIC, self.webcam_cb, 10)
        self.create_timer(1.0 / RATE, self.tick)
        self.get_logger().info(
            f"{WEBCAM_TOPIC}에서 '{DUMMY_CLASS}' -> {CLOUD_TOPIC} 발행")

    def webcam_cb(self, msg):
        found = [(x, y) for cid, _, x, y in items(msg) if cid == DUMMY_CLASS]
        if found:               # 못 본 프레임은 이전 값 유지 (래치)
            self.marks = found
            self.last_seen = self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        # 웹캠이 LOST_TIMEOUT 동안 못 보면 dummy를 치운 걸로 보고 마크 제거.
        # 단, 이미 찍힌 셀은 로봇이 rolling window 밖으로 나가야 실제로 빠진다.
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.marks and expired(self.last_seen, now, LOST_TIMEOUT):
            self.marks = []
            self.get_logger().info(f'웹캠 {LOST_TIMEOUT}s 미검출 — dummy 마크 제거')
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
    assert expired(10.0, 16.0, 5.0) is True         # 6초 경과 > 5 -> 만료
    assert expired(10.0, 14.0, 5.0) is False        # 4초 < 5 -> 아직
    assert expired(None, 99.0, 5.0) is False        # 한 번도 못 봄 -> 만료 아님
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
