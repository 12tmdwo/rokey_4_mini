"""두 로케이터의 map 좌표를 받아 Nav2 goal을 결정하고, 움직이는 차를 쫓는다.

    [탐색] 웹캠이 차를 지정 -> 그쪽으로 접근 (goal 계속 갱신)
      -> [추적] OAK-D가 같은 차를 잡으면 -> OAK-D 좌표로 계속 갱신 = 추적
                이 순간부터 웹캠은 완전 무시 (locked)
      -> [정지+탐색회전] OAK-D가 LOST_TIMEOUT 동안 못 보면 -> goal 취소, 제자리 회전
                하며 재탐색. 다시 잡으면 -> 회전 멈추고 [추적] 재개

Nav2는 주행 중 새 goal을 받으면 선점(preempt)해서 멈추지 않고 목표만 바꾼다.
goal 재전송은 TRACK_RATE로 상한을 건다 (매 검출 프레임마다 쏘면 Nav2가 버벅임).
"""
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from mini_turtle4.detections import match, nearest
from mini_turtle4.nav_controller import Navigator, approach_point, make_pose

# ── 설정 ──────────────────────────────────────────────
WEBCAM_TOPIC = 'webcam/detections'
OAKD_TOPIC = 'oakd/detections'
TARGET_CLASS = 'car'   # 이 클래스만 추적 목표 (dummy 등 장애물은 목표로 안 잡음)
STOP_DIST = 0.5        # 물체 앞 몇 m에서 멈출지
MATCH_RADIUS = 1.0     # 이 반경 + 클래스 일치여야 '같은 물체'로 인정
TRACK_RATE = 3.0       # Hz — goal 재전송 상한 (매 프레임 쏘면 Nav2가 버벅임)
LOST_TIMEOUT = 1.5     # s — OAK-D가 이만큼 연속으로 못 보면 정지 (노이즈 디바운스)
FRAME = 'map'
# ──────────────────────────────────────────────────────


def lost(last_seen, now, timeout):
    """마지막 목격(last_seen) 이후 timeout초 넘게 안 보였으면 True.

    아직 한 번도 못 본 상태(last_seen is None)는 '놓침'이 아니다.
    """
    return last_seen is not None and now - last_seen > timeout


class GoalManager(Node):

    def __init__(self):
        super().__init__('goal_manager_node')
        self.nav = Navigator(self)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.period = 1.0 / TRACK_RATE

        self.target = None      # (class_id, x, y) — 쫓고 있는 물체
        self.locked = False     # OAK-D가 잡은 뒤로 웹캠 갱신 무시
        self.last_send = None   # 마지막 goal 전송 시각 (초) — 스로틀
        self.last_seen = None   # OAK-D가 마지막으로 목표를 본 시각 (초)
        self.stopped = False    # 놓쳐서 정지시킨 상태 (cancel 중복 방지)

        self.create_subscription(Detection3DArray, WEBCAM_TOPIC,
                                 self.webcam_cb, 10)
        self.create_subscription(Detection3DArray, OAKD_TOPIC,
                                 self.oakd_cb, 10)
        self.get_logger().info(
            f'대기 중 — {WEBCAM_TOPIC}로 목표 지정, {OAKD_TOPIC}로 추적')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ── 좌표 입력 ────────────────────────────────────
    def webcam_cb(self, msg):
        if self.locked:
            return                          # OAK-D가 잡은 뒤로 웹캠 완전 무시
        if self.target is None:
            robot = self.robot_xy()
            if robot is None:
                return
            pick = nearest(msg, *robot, TARGET_CLASS)  # 가장 가까운 car (dummy 제외)
            if pick is None:
                return
            self.target = pick
            self.get_logger().info(f"목표 지정: '{pick[0]}' at "
                                   f'({pick[1]:.2f}, {pick[2]:.2f}) [웹캠]')
        elif not self._match_update(msg):
            return                          # 웹캠이 놓침 — 접근 goal 유지
        self.send('웹캠')                   # 탐색 중에도 계속 갱신하며 접근

    def oakd_cb(self, msg):
        if self.target is None:
            return                          # 웹캠이 먼저 목표를 정해야 함
        now = self.now()
        if self._match_update(msg):
            self.locked = True              # 이후 웹캠 무시
            self.last_seen = now
            if self.stopped:
                self.nav.cancel_spin()       # 다시 찾았으니 탐색 회전 중단
                self.get_logger().info('재획득 — 탐색 회전 중단, 추적 재개')
            self.stopped = False            # 다시 봤으니 정지 해제 = 추적 재개
            self.send('OAK-D')
        elif self.locked and not self.stopped and \
                lost(self.last_seen, now, LOST_TIMEOUT):
            self.nav.cancel()               # 놓쳤다 -> 정지
            self.stopped = True
            self.nav.spin_search()          # 제자리 회전하며 재탐색
            self.get_logger().info(f'OAK-D {LOST_TIMEOUT}s 놓침 — 정지 후 탐색 회전')

    def _match_update(self, msg):
        """현재 목표와 같은 물체를 찾아 좌표를 갱신. 찾았으면 True."""
        cls, x, y = self.target
        found = match(msg, x, y, cls, MATCH_RADIUS)
        if found is None:
            return False
        self.target = (cls, *found)
        return True

    # ── goal 계산·전송 ───────────────────────────────
    def send(self, src):
        now = self.now()
        if self.last_send is not None and now - self.last_send < self.period:
            return                          # 스로틀 — TRACK_RATE 상한
        cls, ox, oy = self.target
        robot = self.robot_xy()
        if robot is None:
            return
        rx, ry = robot
        g = approach_point(ox - rx, oy - ry, STOP_DIST)
        if g is None:
            return                          # 이미 STOP_DIST 안 — 새 goal 불필요

        gx, gy, yaw = rx + g[0], ry + g[1], g[2]
        if self.nav.go(make_pose(FRAME, gx, gy, yaw)):
            self.last_send = now
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


def _self_check():
    assert lost(10.0, 12.0, 1.5) is True        # 2초 경과 > 1.5 -> 놓침
    assert lost(10.0, 11.0, 1.5) is False       # 1초 < 1.5 -> 아직 봄
    assert lost(10.0, 11.5, 1.5) is False       # 정확히 1.5 -> 경계, 아직 아님
    assert lost(None, 99.0, 1.5) is False       # 한 번도 못 봄 -> 놓침 아님
    print('goal_manager self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
