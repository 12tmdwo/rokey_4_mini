"""맵 밖 웹캠 YOLO -> bbox 하단 중앙 -> 호모그래피 -> map 좌표 발행.

전제: homography.py로 캘리브레이션해서 homography.npy를 만들어 둘 것.
    ros2 run mini_turtle4 webcam_locator_node
"""
import cv2
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

from mini_turtle4.detections import to_msg
# CAM_INDEX는 homography.py 것을 그대로 쓴다 — 캘리브레이션한 카메라와
# 다른 카메라를 열면 H가 통째로 무의미해지므로 상수를 두 곳에 두지 않는다.
from mini_turtle4.homography import CAM_INDEX, bottom_center, load, to_map

# ── 설정 ──────────────────────────────────────────────
H_PATH = '/home/rokey/turtlebot4_ws/homography.npy'
MODEL_PATH = '/home/rokey/turtlebot4_ws/best.pt'
CONF = 0.6
RATE = 5.0                  # Hz
TOPIC = 'webcam/detections'
FRAME = 'map'
SHOW = True                 # 탐지 화면 창 표시
# ──────────────────────────────────────────────────────


class WebcamLocator(Node):

    def __init__(self):
        super().__init__('webcam_locator_node')
        from ultralytics import YOLO
        self.H = load(H_PATH)
        self.model = YOLO(MODEL_PATH)
        self.cap = cv2.VideoCapture(CAM_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError(f'카메라 {CAM_INDEX}를 열 수 없습니다.')

        self.pub = self.create_publisher(Detection3DArray, TOPIC, 10)
        self.create_timer(1.0 / RATE, self.tick)
        self.get_logger().info(f'{H_PATH} + {MODEL_PATH} 로드 — {TOPIC} 발행')

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('프레임 읽기 실패')
            return
        # undistort 안 함 (체커보드 없음). 렌즈 왜곡은 화면 가장자리에서 커지므로
        # 물건을 화면 중앙 쪽에 두고 쓸 것. 필요해지면 여기서 frame을 펴고 YOLO.

        # agnostic_nms: 같은 물체에 car/dommy가 동시에 붙는 걸 막는다.
        # goal_manager가 클래스 이름으로 목표를 매칭하므로 라벨이 흔들리면 안 된다.
        result = self.model(frame, conf=CONF, agnostic_nms=True, verbose=False)[0]

        found = []
        for b in result.boxes:
            u, v = bottom_center(*b.xyxy[0].tolist())
            x, y = to_map(self.H, u, v)
            name = self.model.names[int(b.cls)]
            found.append((name, float(b.conf), x, y))
            self.get_logger().info(
                f'{name:12s} px({u:6.0f},{v:6.0f}) -> map({x:6.2f},{y:6.2f})')
        self.pub.publish(to_msg(FRAME, self.get_clock().now().to_msg(), found))

        if SHOW:
            cv2.imshow('webcam yolo', result.plot())
            cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = WebcamLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
