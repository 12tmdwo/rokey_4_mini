import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')
        model_path = self.declare_parameter('model_path', 'yolov8n.pt').value
        self.conf = self.declare_parameter('conf', 0.6).value
        self.model = YOLO(model_path)
        self.get_logger().info(f'loaded {model_path}')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/yolo/image', 10)
        self.create_subscription(Image, '/oakd/rgb/preview/image_raw',
                                 self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        result = self.model(img, conf=self.conf, verbose=False)[0]
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
