import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from mini_turtle4.depth_math import decode_depth, depth_at, to_depth_px
from mini_turtle4.paths import MODEL

# ── 설정 ──────────────────────────────────────────────
# rgb/image_raw는 로봇에서 depth와 동일하게 맞춰져 있다 (둘 다 704x704, 같은 K:
# fx=569.95 cx=353.40). 그래서 to_depth_px가 1:1 그대로 통과한다. preview(320)를
# 쓰면 2.2배 스케일이 붙으므로, 정렬해둔 image_raw를 쓰는 게 맞다.
RGB_TOPIC = 'oakd/rgb/image_raw/compressed'
# depth는 16UC1이라 jpeg가 안 된다 -> png 기반 compressedDepth 사용 (decode_depth).
DEPTH_TOPIC = 'oakd/stereo/image_raw/compressedDepth'
# ──────────────────────────────────────────────────────


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')
        model_path = self.declare_parameter('model_path', MODEL).value
        self.conf = self.declare_parameter('conf', 0.6).value
        self.model = YOLO(model_path)
        self.get_logger().info(f'loaded {model_path}')

        self.bridge = CvBridge()
        self.depth = None
        self.pub = self.create_publisher(Image, 'yolo/image', 10)
        self.create_subscription(CompressedImage, DEPTH_TOPIC,
                                 self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, RGB_TOPIC,
                                 self.cb, qos_profile_sensor_data)

    def depth_cb(self, msg):
        self.depth = decode_depth(msg.data, msg.format)

    def cb(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        result = self.model(img, conf=self.conf, verbose=False)[0]

        if self.depth is not None:
            for b in result.boxes:
                u, v = b.xywh[0][:2].tolist()           # bbox 중심
                du, dv = to_depth_px(u, v, img.shape, self.depth.shape)
                z = depth_at(self.depth, du, dv)
                name = self.model.names[int(b.cls)]
                # ponytail: 광축(Z) 거리라 화면 가장자리 물체는 실제 직선거리보다
                # 짧게 나온다. 정확히 필요하면 oakd_locator_node의 deproject 사용.
                self.get_logger().info(
                    f'{name:12s} {z * 100:6.1f}cm' if z
                    else f'{name:12s} depth 무효')

        out = self.bridge.cv2_to_imgmsg(result.plot(), 'bgr8')
        out.header = msg.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
