"""로케이터들의 Detection3DArray를 RViz용 MarkerArray로 재발행.

RViz2엔 Detection3DArray 기본 디스플레이가 없어서 그대로는 안 보인다. map 좌표를
구(위치) + 글자(클래스 라벨) 마커로 다시 쏴서, RViz의 MarkerArray 디스플레이
(topic: detections/markers)로 차 위치를 실시간으로 본다. 안 갱신되면 LIFETIME 뒤
알아서 사라진다 (차가 화면 밖으로 나가면 마커도 사라짐).
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray

from mini_turtle4.detections import items

# ── 설정 ──────────────────────────────────────────────
TOPICS = ['webcam/detections', 'oakd/detections']
OUT = 'detections/markers'
LIFETIME = 0.5              # s — 이 시간 안 갱신 안 되면 RViz에서 사라짐
SIZE = 0.2                 # m — 구 지름
COLORS = {'car': (0.0, 1.0, 0.0), 'dommy': (1.0, 0.0, 0.0)}   # 나머지는 흰색
# ──────────────────────────────────────────────────────


class DetectionMarkers(Node):

    def __init__(self):
        super().__init__('detection_marker_node')
        self.life = Duration(seconds=LIFETIME).to_msg()
        self.pub = self.create_publisher(MarkerArray, OUT, 10)
        for t in TOPICS:
            ns = t.split('/')[0]        # webcam / oakd — 소스별 마커 id 분리
            self.create_subscription(
                Detection3DArray, t,
                lambda msg, ns=ns: self.cb(msg, ns), 10)
        self.get_logger().info(f'{TOPICS} -> {OUT} 마커 발행')

    def cb(self, msg, ns):
        arr = MarkerArray()
        for i, (cls, _score, x, y) in enumerate(items(msg)):
            arr.markers.append(self.dot(msg.header, ns, i * 2, cls, x, y))
            arr.markers.append(self.text(msg.header, ns, i * 2 + 1, cls, x, y))
        if arr.markers:
            self.pub.publish(arr)

    def _base(self, header, ns, mid, cls, x, y):
        m = Marker()
        m.header = header           # frame_id='map', 로케이터가 채워 보냄
        m.ns = ns
        m.id = mid
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.w = 1.0
        r, g, b = COLORS.get(cls, (1.0, 1.0, 1.0))
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9
        m.lifetime = self.life
        return m

    def dot(self, header, ns, mid, cls, x, y):
        m = self._base(header, ns, mid, cls, x, y)
        m.type = Marker.SPHERE
        m.scale.x = m.scale.y = m.scale.z = SIZE
        return m

    def text(self, header, ns, mid, cls, x, y):
        m = self._base(header, ns, mid, cls, x, y)
        m.type = Marker.TEXT_VIEW_FACING
        m.text = f'{ns}:{cls}'
        m.pose.position.z = 0.3     # 구 위에 라벨
        m.scale.z = 0.15            # 글자 높이
        return m


def main():
    rclpy.init()
    node = DetectionMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
