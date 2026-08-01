"""픽셀 -> map 좌표 호모그래피.

캘리브레이션:  python3 homography.py           (창에서 바닥 4점 클릭)
사용:          H = load(); x, y = to_map(H, u, v)
"""
import sys

import cv2
import numpy as np

from mini_turtle4.paths import H_FILE

# 여기만 채우세요. 클릭하는 순서와 같은 순서로 map 프레임 실측 좌표 (미터).
# 맵 네 귀퉁이처럼 넓게 분산시킬 것.
MAP_POINTS = [
    (3.26, 1.41),   # 1번째 클릭 지점의 map 좌표
    (1.06, 0.95),   # 2번째
    (0.62, -0.35),   # 3번째
    (3.10, -0.51),   # 4번째
]

CAM_INDEX = 2


def to_map(H, u, v):
    """픽셀 (u, v) -> map (x, y). bbox 하단 중앙 픽셀을 넣을 것."""
    p = cv2.perspectiveTransform(np.array([[[u, v]]], np.float32), H)
    return float(p[0, 0, 0]), float(p[0, 0, 1])


def bottom_center(x1, y1, x2, y2):
    """bbox(xyxy) -> 물건이 바닥에 닿는 픽셀 = 하단 중앙.

    호모그래피는 바닥 평면 기준이라 bbox 중심을 쓰면 물건 높이만큼 밀린다.
    """
    return (x1 + x2) / 2, y2


def load(path=H_FILE):
    return np.load(path)


def calibrate():
    if len(set(MAP_POINTS)) < 4:
        sys.exit('MAP_POINTS를 먼저 채우세요 (서로 다른 4점).')

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        sys.exit(f'카메라 {CAM_INDEX}를 열 수 없습니다.')

    pixels = []
    cv2.namedWindow('calib')
    cv2.setMouseCallback(
        'calib',
        lambda e, x, y, *_: e == cv2.EVENT_LBUTTONDOWN and len(pixels) < 4
        and pixels.append((x, y)))

    print('바닥 4점을 MAP_POINTS 순서대로 클릭. r=초기화, q=취소')
    while len(pixels) < 4:
        ok, frame = cap.read()
        if not ok:
            sys.exit('프레임 읽기 실패.')
        for i, (x, y) in enumerate(pixels):
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(frame, f'{i + 1} {MAP_POINTS[i]}', (x + 8, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow('calib', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            sys.exit('취소됨.')
        if key == ord('r'):
            pixels.clear()

    cap.release()
    cv2.destroyAllWindows()

    H, _ = cv2.findHomography(np.array(pixels, np.float32),
                              np.array(MAP_POINTS, np.float32))
    if H is None:
        sys.exit('H 계산 실패. 4점이 일직선상은 아닌지 확인하세요.')
    np.save(H_FILE, H)

    print(f'{H_FILE} 저장. 검증 (클릭점 -> map, 오차):')
    for (u, v), truth in zip(pixels, MAP_POINTS):
        x, y = to_map(H, u, v)
        err = np.hypot(x - truth[0], y - truth[1])
        print(f'  ({u:4d},{v:4d}) -> ({x:6.3f},{y:6.3f})  실측{truth}  {err * 100:.1f}cm')


def _self_check():
    """알려진 사각형으로 H를 만들어 왕복 변환이 맞는지 확인."""
    px = np.array([(100, 100), (500, 100), (500, 400), (100, 400)], np.float32)
    mp = np.array([(0, 0), (2, 0), (2, 1.5), (0, 1.5)], np.float32)
    H, _ = cv2.findHomography(px, mp)
    for (u, v), (x, y) in zip(px, mp):
        gx, gy = to_map(H, u, v)
        assert abs(gx - x) < 1e-6 and abs(gy - y) < 1e-6, (gx, gy, x, y)
    cx, cy = to_map(H, 300, 250)
    assert abs(cx - 1.0) < 1e-6 and abs(cy - 0.75) < 1e-6, (cx, cy)

    # bbox -> 하단 중앙 -> map (노드가 쓰는 경로 전체)
    assert bottom_center(100, 100, 500, 400) == (300.0, 400.0)
    bx, by = to_map(H, *bottom_center(100, 100, 500, 400))
    assert abs(bx - 1.0) < 1e-6 and abs(by - 1.5) < 1e-6, (bx, by)
    print('self-check ok')


if __name__ == '__main__':
    _self_check() if '--test' in sys.argv else calibrate()
