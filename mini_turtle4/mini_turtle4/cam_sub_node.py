import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CamSub(Node):

    def __init__(self):
        super().__init__('cam_sub_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/oakd/rgb/preview/image_raw',
                                 self.rgb_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/oakd/stereo/image_raw',
                                 self.depth_cb, qos_profile_sensor_data)

    def rgb_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.get_logger().info(f'rgb {img.shape}')

    def depth_cb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg)  # 16UC1, mm
        h, w = depth.shape
        self.get_logger().info(f'center depth: {depth[h // 2, w // 2]} mm')


def main():
    rclpy.init()
    node = CamSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
