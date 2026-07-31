"""두 로케이터의 map 좌표를 받아 Nav2 goal을 결정하고 갱신한다.

    웹캠이 물체를 지정 -> goal 전송 (대략 이동)
      -> 주행 중 OAK-D가 같은 물체를 잡으면 -> 더 정확한 좌표로 goal 갱신
      -> OAK-D가 못 보면 (너무 가깝거나 화면 밖) -> 마지막 goal 유지

Nav2는 주행 중 새 goal을 받으면 선점(preempt)해서 멈추지 않고 목표만 바꾼다.
"""
import math

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from mini_turtle4.detections import match, nearest
from mini_turtle4.nav_controller import Navigator, approach_point, make_pose

# ── 설정 ──────────────────────────────────────────────
WEBCAM_TOPIC = 'webcam/detections'
OAKD_TOPIC = 'oakd/detections'
STOP_DIST = 0.5        # 물체 앞 몇 m에서 멈출지
UPDATE_THRESH = 0.15   # 물체 좌표가 이만큼 넘게 움직여야 goal 갱신
MATCH_RADIUS = 1.0     # 이 반경 + 클래스 일치여야 '같은 물체'로 인정
FRAME = 'map'
# ──────────────────────────────────────────────────────


class GoalManager(Node):

    def __init__(self):
        super().__init__('goal_manager_node')
        self.nav = Navigator(self)
        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.target = None      # (class_id, x, y) — 쫓고 있는 물체
        self.sent = None        # 마지막으로 goal을 보낸 시점의 물체 좌표
        self.locked = False     # OAK-D가 잡은 뒤로는 웹캠 갱신 무시

        self.create_subscription(Detection3DArray, WEBCAM_TOPIC,
                                 self.webcam_cb, 10)
        self.create_subscription(Detection3DArray, OAKD_TOPIC,
                                 self.oakd_cb, 10)
        self.get_logger().info(
            f'대기 중 — {WEBCAM_TOPIC}로 목표 지정, {OAKD_TOPIC}로 보정')

    # ── 좌표 입력 ────────────────────────────────────
    def webcam_cb(self, msg):
        if self.target is None:
            robot = self.robot_xy()
            if robot is None:
                return
            pick = nearest(msg, *robot)     # 로봇에서 가장 가까운 물체
            if pick is None:
                return
            self.target = pick
            self.get_logger().info(f"목표 지정: '{pick[0]}' at "
                                   f'({pick[1]:.2f}, {pick[2]:.2f}) [웹캠]')
            self.send()
        elif not self.locked:
            self.refine(msg, '웹캠')

    def oakd_cb(self, msg):
        if self.target is None:
            return                          # 웹캠이 먼저 목표를 정해야 함
        if self.refine(msg, 'OAK-D'):
            self.locked = True              # 이후 웹캠 갱신은 무시

    def refine(self, msg, src):
        """현재 목표와 같은 물체를 찾아 좌표를 갱신. 찾았으면 True."""
        cls, x, y = self.target
        found = match(msg, x, y, cls, MATCH_RADIUS)
        if found is None:
            return False                    # 못 봤으면 마지막 goal 유지
        self.target = (cls, *found)
        self.send(src)
        return True

    # ── goal 계산·전송 ───────────────────────────────
    def send(self, src='웹캠'):
        cls, ox, oy = self.target
        if self.sent and math.hypot(ox - self.sent[0], oy - self.sent[1]) \
                < UPDATE_THRESH:
            return                          # 요동 수준 — 갱신 생략

        robot = self.robot_xy()
        if robot is None:
            return
        rx, ry = robot
        g = approach_point(ox - rx, oy - ry, STOP_DIST)
        if g is None:
            self.get_logger().info(f'이미 {STOP_DIST}m 안 — 주행 생략')
            self.sent = (ox, oy)
            return

        gx, gy, yaw = rx + g[0], ry + g[1], g[2]
        if self.nav.go(make_pose(FRAME, gx, gy, yaw)):
            self.sent = (ox, oy)
            self.get_logger().info(
                f"[{src}] '{cls}' ({ox:.2f}, {oy:.2f}) 앞 {STOP_DIST}m")

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform(FRAME, 'base_link',
                                         rclpy.time.Time()).transform.translation
            return t.x, t.y
        except TransformException as e:
            self.get_logger().warn(f'TF 실패 ({FRAME}->base_link): {e}',
                                   throttle_duration_sec=5.0)
            return None


def main():
    rclpy.init()
    node = GoalManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
