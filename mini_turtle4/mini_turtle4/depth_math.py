"""depth 카메라 좌표 계산. ROS 무관, 노드가 import해서 사용."""
import numpy as np

PATCH = 5   # depth 샘플 반경 (픽셀). 한 점만 보면 0(무효)이 잘 나옴


def to_depth_px(u, v, rgb_shape, depth_shape):
    """RGB 픽셀 -> depth 픽셀. 두 이미지 해상도가 달라서 필요."""
    # 실측 확인(robot4, RGBD 파이프라인): RGB 320x320, depth 704x704로 둘 다
    # 같은 FOV의 정사각형이고 depth가 RGB 프레임에 정렬돼 있다.
    # RGB 중심(160,160) -> (352,352) vs depth 주점(353.4,354.1) = 1px 오차.
    # 카메라 설정이 바뀌면 depth_checker로 다시 확인할 것.
    rh, rw = rgb_shape[:2]
    dh, dw = depth_shape[:2]
    return int(u / rw * dw), int(v / rh * dh)


def depth_at(depth_mm, u, v, patch=PATCH):
    """(u, v) 주변 patch의 유효값 중앙값 -> 미터. 유효값 없으면 None."""
    h, w = depth_mm.shape
    win = depth_mm[max(0, v - patch):min(h, v + patch + 1),
                   max(0, u - patch):min(w, u + patch + 1)]
    win = win[win > 0]
    return float(np.median(win)) / 1000.0 if win.size else None


def deproject(u, v, z, K):
    """depth 픽셀 + 거리 -> 카메라 광학 프레임 3D (x우, y하, z전방)."""
    return ((u - K[0, 2]) * z / K[0, 0],
            (v - K[1, 2]) * z / K[1, 1],
            z)


def _self_check():
    d = np.zeros((400, 640), np.uint16)
    d[195:205, 315:325] = 1500                 # 중앙에 1.5m 블록
    assert depth_at(d, 320, 200) == 1.5
    assert depth_at(d, 10, 10) is None         # 전부 0 -> 무효
    assert depth_at(d, 0, 0) is None           # 경계에서 안 터짐
    assert depth_at(d, 639, 399) is None

    assert to_depth_px(125, 125, (250, 250), (400, 640)) == (320, 200)
    assert to_depth_px(0, 0, (250, 250), (400, 640)) == (0, 0)

    K = np.array([[500., 0., 320.], [0., 500., 200.], [0., 0., 1.]])
    assert deproject(320, 200, 2.0, K) == (0.0, 0.0, 2.0)   # 정중앙
    x, y, z = deproject(420, 200, 2.0, K)                   # 오른쪽 100px
    assert abs(x - 0.4) < 1e-9 and abs(y) < 1e-9 and z == 2.0
    print('depth_math self-check ok')


if __name__ == '__main__':
    _self_check()
