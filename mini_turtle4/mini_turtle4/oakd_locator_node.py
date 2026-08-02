"""OAK-D YOLO + depth -> 물체의 map 좌표 발행. 주행은 하지 않는다.

주행은 goal_manager_node가 맡는다.
"""
import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  PointStamped TF 변환 등록
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from mini_turtle4.depth_math import decode_depth, depth_at, deproject, to_depth_px
from mini_turtle4.detections import to_msg
from mini_turtle4.paths import MODEL as MODEL_PATH

# ── 설정 ──────────────────────────────────────────────
# compressed로 받아 WiFi 부담을 줄인다. rgb/image_raw는 depth와 동일 704x704라
# to_depth_px가 1:1 통과 (yolo_node와 동일 근거). camera_info는 작아서 raw 그대로.
RGB_TOPIC = 'oakd/rgb/image_raw/compressed'
DEPTH_TOPIC = 'oakd/stereo/image_raw/compressedDepth'
DEPTH_INFO_TOPIC = 'oakd/stereo/camera_info'
CONF = 0.6
TOPIC = 'oakd/detections'
FRAME = 'map'
SHOW = True                 # 탐지 화면 창 표시 (거리 + map 좌표 같이 표기)
# ──────────────────────────────────────────────────────


class OakdLocator(Node):

    def __init__(self):
        super().__init__('oakd_locator_node')
        from ultralytics import YOLO
        self.model = YOLO(MODEL_PATH)
        self.bridge = CvBridge()
        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.depth = None
        self.depth_frame = None
        self.K = None

        self.pub = self.create_publisher(Detection3DArray, TOPIC, 10)
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC,
                                 self.info_cb, 10)
        self.create_subscription(CompressedImage, DEPTH_TOPIC,
                                 self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, RGB_TOPIC,
                                 self.rgb_cb, qos_profile_sensor_data)
        self.get_logger().info(f'{MODEL_PATH} 로드 — {TOPIC} 발행')

    def info_cb(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def depth_cb(self, msg):
        self.depth = decode_depth(msg.data, msg.format)
        self.depth_frame = msg.header.frame_id

    def rgb_cb(self, msg):
        if self.depth is None or self.K is None:
            return

        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        # agnostic_nms: webcam_locator_node와 같은 이유 (라벨 중복 방지)
        result = self.model(img, conf=CONF, agnostic_nms=True,
                            verbose=False)[0]

        found = []
        marks = []          # 화면에 덧그릴 (x, y, 문구)
        for b in result.boxes:
            u, v = b.xywh[0][:2].tolist()           # bbox 중심
            x1, y1 = b.xyxy[0][:2].tolist()
            du, dv = to_depth_px(u, v, img.shape, self.depth.shape)
            z = depth_at(self.depth, du, dv)
            if not z:
                # depth 무효 = 너무 가깝거나 반사. 화면에는 이유를 남긴다.
                marks.append((x1, y1, 'depth 무효'))
                continue
            pt = PointStamped()
            pt.header.frame_id = self.depth_frame
            pt.point.x, pt.point.y, pt.point.z = deproject(du, dv, z, self.K)
            try:
                p = self.tf.transform(pt, FRAME).point
            except TransformException as e:
                self.get_logger().warn(f'TF 실패 ({self.depth_frame}->{FRAME}): {e}')
                return
            name = self.model.names[int(b.cls)]
            found.append((name, float(b.conf), p.x, p.y))
            marks.append((x1, y1, f'{z:.2f}m ({p.x:.2f},{p.y:.2f})'))
            self.get_logger().info(
                f'{name:12s} {z:.2f}m -> map({p.x:6.2f},{p.y:6.2f})')

        self.pub.publish(to_msg(FRAME, msg.header.stamp, found))

        if SHOW:
            view = result.plot()
            for x, y, text in marks:
                cv2.putText(view, text, (int(x), int(y) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            cv2.imshow('oakd yolo', view)
            cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = OakdLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
