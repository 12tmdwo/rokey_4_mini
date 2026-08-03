"""로봇 이동 제어 (Nav2). 노드에서 import해서 사용.

    nav = Navigator(node)
    nav.go(make_pose('map', x, y, yaw))
"""
import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient


def approach_point(x, y, stop_dist):
    """base_link 기준 물체 (x, y) -> 물체 앞 stop_dist 지점 (gx, gy, yaw).

    물체를 바라보는 방향으로 yaw를 잡는다. 이미 stop_dist 안이면 None.
    """
    d = math.hypot(x, y)
    if d <= stop_dist:
        return None
    r = (d - stop_dist) / d
    return x * r, y * r, math.atan2(y, x)


def make_pose(frame, x, y, yaw):
    """(x, y, yaw) -> PoseStamped. stamp는 0 = TF 최신값 사용."""
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p


class Navigator:
    """NavigateToPose + Spin(제자리 회전 탐색) 액션 래퍼. 한 번에 goal 하나만."""

    def __init__(self, node, action='navigate_to_pose', spin_action='spin'):
        self.log = node.get_logger()
        self.client = ActionClient(node, NavigateToPose, action)
        self.active = False
        self.handle = None

        self.spin_client = ActionClient(node, Spin, spin_action)
        self.spin_handle = None
        self._want_spin = False  # cancel_spin() 전까지 계속 회전 (아래 _spin_done 참고)

    def go(self, pose):
        """goal 전송 (비동기). 서버가 받아들이면 True.

        주행 중에 다시 불러도 된다. Nav2가 선점(preempt)해서 멈추지 않고
        목표만 갈아끼운다 (bt_action_server_impl.hpp의 on_preempt_callback).
        """
        # 콜백 안에서 wait_for_server를 쓰면 단일 스레드 executor가 막힌다
        if not self.client.server_is_ready():
            self.log.warn('navigate_to_pose 서버 아직 없음 — Nav2가 켜져 있나요?')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose
        verb = '갱신' if self.active else '전송'
        self.active = True
        self.client.send_goal_async(goal).add_done_callback(self._accepted)
        self.log.info(f'goal {verb}: ({pose.pose.position.x:.2f}, '
                      f'{pose.pose.position.y:.2f}) @ {pose.header.frame_id}')
        return True

    def cancel(self):
        if self.handle:
            self.handle.cancel_goal_async()

    def _accepted(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.log.error('goal 거부됨')
            self.active = False
            return
        self.handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._done(f, handle))

    def _done(self, fut, handle):
        if handle is not self.handle:
            return              # 선점당한 예전 goal — 무시
        self.active = False
        self.handle = None
        # status 4 = SUCCEEDED
        self.log.info(f'주행 종료 (status={fut.result().status})')

    # ── 제자리 회전 탐색 (놓친 물체 재획득용) ──────────
    def spin_search(self, angle=2 * math.pi, time_allowance=30):
        """물체를 놓쳤을 때 제자리 회전하며 찾는다. 이미 도는 중이면 무시.

        한 바퀴(angle) 다 돌아도 못 찾으면 _spin_done에서 스스로 다시 돈다 —
        cancel_spin()으로 명시적으로 멈출 때까지 계속 (재획득은 호출 쪽 책임).
        """
        if self._want_spin:
            return
        self._want_spin = True
        self._send_spin(angle, time_allowance)

    def cancel_spin(self):
        """탐색 회전을 멈춘다 (물체를 다시 찾았을 때 호출)."""
        self._want_spin = False
        if self.spin_handle:
            self.spin_handle.cancel_goal_async()
            self.spin_handle = None

    def _send_spin(self, angle, time_allowance):
        if not self.spin_client.server_is_ready():
            self.log.warn('spin 서버 아직 없음 — Nav2 behavior_server가 켜져 있나요?',
                          throttle_duration_sec=5.0)
            return
        goal = Spin.Goal()
        goal.target_yaw = angle
        goal.time_allowance = Duration(sec=time_allowance)
        self.spin_client.send_goal_async(goal).add_done_callback(
            lambda fut: self._spin_accepted(fut, angle, time_allowance))
        self.log.info('탐색 회전 시작')

    def _spin_accepted(self, fut, angle, time_allowance):
        handle = fut.result()
        if not handle.accepted:
            self.log.error('spin 거부됨')
            self._want_spin = False
            return
        self.spin_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._spin_done(f, handle, angle, time_allowance))

    def _spin_done(self, fut, handle, angle, time_allowance):
        if handle is not self.spin_handle:
            return              # 이미 취소되고 다른 회전으로 교체됨 — 무시
        self.spin_handle = None
        if self._want_spin:      # 한 바퀴 다 돌았는데 아직 못 찾음 -> 계속 회전
            self._send_spin(angle, time_allowance)


def _self_check():
    x, y, yaw = approach_point(2.0, 0.0, 0.5)          # 정면 2m
    assert abs(x - 1.5) < 1e-9 and abs(y) < 1e-9 and abs(yaw) < 1e-9

    x, y, yaw = approach_point(0.0, 2.0, 0.5)          # 왼쪽 2m
    assert abs(x) < 1e-9 and abs(y - 1.5) < 1e-9
    assert abs(yaw - math.pi / 2) < 1e-9

    x, y, _ = approach_point(3.0, 4.0, 1.0)            # d=5 -> r=0.8
    assert abs(x - 2.4) < 1e-9 and abs(y - 3.2) < 1e-9

    assert approach_point(0.3, 0.0, 0.5) is None       # 이미 가까움

    p = make_pose('base_link', 1.0, 0.0, math.pi / 2)
    assert abs(p.pose.orientation.z - math.sqrt(0.5)) < 1e-9
    assert abs(p.pose.orientation.w - math.sqrt(0.5)) < 1e-9
    print('nav_controller self-check ok')


if __name__ == '__main__':
    _self_check()
