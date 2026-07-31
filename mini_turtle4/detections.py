"""Detection3DArray 조립·해석·매칭. 로케이터 두 개와 goal 노드가 공용.

좌표는 map 프레임 평면 (z=0), 클래스 이름은 hypothesis.class_id에 실린다.
"""
import math

from vision_msgs.msg import (Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)


def to_msg(frame, stamp, found):
    """found = [(class_id, score, x, y)] -> Detection3DArray."""
    msg = Detection3DArray()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    for class_id, score, x, y in found:
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = str(class_id)
        h.hypothesis.score = float(score)
        h.pose.pose.position.x = float(x)
        h.pose.pose.position.y = float(y)
        h.pose.pose.orientation.w = 1.0
        d = Detection3D()
        d.header = msg.header
        d.results.append(h)
        d.bbox.center = h.pose.pose
        msg.detections.append(d)
    return msg


def items(msg):
    """Detection3DArray -> [(class_id, score, x, y)]."""
    return [(d.results[0].hypothesis.class_id,
             d.results[0].hypothesis.score,
             d.results[0].pose.pose.position.x,
             d.results[0].pose.pose.position.y)
            for d in msg.detections if d.results]


def nearest(msg, x, y):
    """(x, y)에서 가장 가까운 탐지 -> (class_id, ox, oy). 없으면 None."""
    best = min(((math.hypot(ox - x, oy - y), cid, ox, oy)
                for cid, _, ox, oy in items(msg)), default=None)
    return best[1:] if best else None


def match(msg, x, y, class_id, radius):
    """(x, y) 반경 안에서 class_id가 같은 것 중 가장 가까운 것 -> (ox, oy).

    없으면 None. 다른 물건으로 목표가 갈아타는 걸 막는 게 목적.
    """
    best = min(((math.hypot(ox - x, oy - y), ox, oy)
                for cid, _, ox, oy in items(msg)
                if cid == class_id and math.hypot(ox - x, oy - y) <= radius),
               default=None)
    return best[1:] if best else None


def _self_check():
    import builtin_interfaces.msg as bi
    stamp = bi.Time()
    msg = to_msg('map', stamp, [('bottle', 0.9, 1.0, 0.0),
                                ('cup', 0.8, 3.0, 0.0),
                                ('bottle', 0.7, 1.2, 0.0)])
    assert msg.header.frame_id == 'map'
    assert len(msg.detections) == 3
    assert items(msg)[0] == ('bottle', 0.9, 1.0, 0.0)

    assert nearest(msg, 0.0, 0.0) == ('bottle', 1.0, 0.0)
    assert nearest(msg, 5.0, 0.0) == ('cup', 3.0, 0.0)
    assert nearest(to_msg('map', stamp, []), 0.0, 0.0) is None

    # 반경 안 + 클래스 일치 -> 가장 가까운 bottle
    assert match(msg, 1.05, 0.0, 'bottle', 0.5) == (1.0, 0.0)
    assert match(msg, 1.25, 0.0, 'bottle', 0.5) == (1.2, 0.0)
    # 클래스가 다르면 반경 안이어도 무시
    assert match(msg, 3.0, 0.0, 'bottle', 0.5) is None
    # 반경 밖이면 클래스가 같아도 무시
    assert match(msg, 9.0, 0.0, 'bottle', 0.5) is None
    print('detections self-check ok')


if __name__ == '__main__':
    _self_check()
