"""웹캠이 YOLO로 잡은 dummy(map 좌표)를 PointCloud2로 발행 -> Nav2 코스트맵 장애물.

RPLIDAR가 dummy를 못 잡아서 로봇이 밀고 지나가는 걸 막는다. goal 결정은
goal_manager, 회피는 코스트맵 — 레이어가 달라 서로 간섭 없다.

웹캠이 잠깐 놓쳐도 마지막 좌표를 유지하고, LOST_TIMEOUT 동안 연속으로 못 보면
발행점을 비운 뒤 local/global 코스트맵을 한 번씩 초기화한다.
"""
import rclpy
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray

from mini_turtle4.detections import items

# ── 설정 ──────────────────────────────────────────────
DETECT_TOPIC = 'webcam/detections'
CLOUD_TOPIC = 'dummy_cloud'
DUMMY_CLASS = 'dommy'       # 모델 학습 라벨 (오타 아님)
DUMMY_RADIUS = 0.1         # m — dummy 실제 반경. inflation 0.45m가 더 붙음
SPACING = 0.05              # m — map 해상도. 원판을 이 격자로 채운다
RATE = 2.0                  # Hz — 감지 중에는 마지막 좌표를 낮은 주기로 재발행
LOST_TIMEOUT = 1.5          # s — 이 시간 동안 연속 미검출이면 제거
Z = 0.2                     # m — voxel 높이 밴드(0~0.8) 안
FRAME = 'map'
CLEAR_SERVICES = (
    'local_costmap/clear_entirely_local_costmap',
    'global_costmap/clear_entirely_global_costmap',
)
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


def lost(last_seen, now, timeout):
    return last_seen is not None and now - last_seen > timeout


class DummyObstacle(Node):

    def __init__(self):
        super().__init__('dummy_obstacle_node')
        self.marks = []         # 마지막으로 본 dummy 좌표 [(x,y)]
        self.last_seen = None
        self.loss_handled = True
        self.clear_clients = {
            name: self.create_client(ClearEntireCostmap, name)
            for name in CLEAR_SERVICES
        }
        self.pending_clears = set()
        self.pub = self.create_publisher(PointCloud2, CLOUD_TOPIC, 10)
        self.create_subscription(Detection3DArray, DETECT_TOPIC, self.cb, 10)
        self.create_timer(1.0 / RATE, self.tick)
        self.get_logger().info(
            f"{DETECT_TOPIC}에서 '{DUMMY_CLASS}' -> {CLOUD_TOPIC} 발행")

    def cb(self, msg):
        found = [(x, y) for cid, _, x, y in items(msg) if cid == DUMMY_CLASS]
        if found:               # 한두 프레임 누락은 tick의 시간 판정이 흡수한다.
            self.marks = found
            self.last_seen = self.get_clock().now().nanoseconds * 1e-9
            self.loss_handled = False

    def tick(self):
        just_lost = False
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.loss_handled and lost(self.last_seen, now, LOST_TIMEOUT):
            self.marks = []
            self.pending_clears.update(self.clear_clients)
            self.loss_handled = True
            just_lost = True
            self.get_logger().info(
                f'dummy {LOST_TIMEOUT}s 미검출 — 코스트맵에서 제거')

        pts = [p for cx, cy in self.marks
               for p in disc_points(cx, cy, Z, DUMMY_RADIUS, SPACING)]
        h = Header()
        h.frame_id = FRAME
        h.stamp = self.get_clock().now().to_msg()
        self.pub.publish(create_cloud_xyz32(h, pts))

        # 빈 cloud가 코스트맵에 먼저 도착하도록 유실 다음 주기부터 초기화한다.
        if not just_lost:
            for name in tuple(self.pending_clears):
                client = self.clear_clients[name]
                if client.service_is_ready():
                    client.call_async(ClearEntireCostmap.Request())
                    self.pending_clears.remove(name)
                    self.get_logger().info(f'코스트맵 초기화 요청: {name}')


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
    assert not lost(None, 10.0, 1.5)
    assert not lost(10.0, 11.5, 1.5)
    assert lost(10.0, 11.5001, 1.5)
    print('dummy_obstacle self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
