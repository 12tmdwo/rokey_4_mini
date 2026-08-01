"""맵 이미지를 클릭해서 map 좌표를 읽는다. ROS 실행 없이 동작.

    python3 map_point_picker.py            (MAP_YAML 사용)
    python3 map_point_picker.py 다른맵.yaml

호모그래피용 4점을 고를 때 쓸 것. 찍은 순서대로 MAP_POINTS 형식으로 출력된다.
"""
import sys

import cv2
import numpy as np
import yaml

from mini_turtle4.paths import MAP_YAML

SCALE = 6           # 화면 확대 배율 (맵 픽셀이 5cm라 그냥 보면 너무 작다)


def load_map(path):
    """yaml -> (이미지, resolution, origin). 이미지는 pgm 원본 그대로."""
    with open(path) as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not img_path.startswith('/'):
        img_path = path.rsplit('/', 1)[0] + '/' + img_path
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f'맵 이미지를 못 읽음: {img_path}')
    return img, meta['resolution'], meta['origin']


def to_world(px, py, height, resolution, origin):
    """이미지 픽셀 -> map 좌표. 이미지는 위가 +y라 세로를 뒤집는다."""
    return (origin[0] + (px + 0.5) * resolution,
            origin[1] + (height - py - 0.5) * resolution)


def pick(path=MAP_YAML):
    img, res, origin = load_map(path)
    h, w = img.shape
    print(f'{path}: {w}x{h}px, {res}m/px, origin={origin[:2]}')
    print(f'범위: x {origin[0]:.2f}~{origin[0] + w * res:.2f}, '
          f'y {origin[1]:.2f}~{origin[1] + h * res:.2f}')
    print('바닥 4점을 클릭. r=초기화, q=종료')

    big = cv2.cvtColor(cv2.resize(img, (w * SCALE, h * SCALE),
                                  interpolation=cv2.INTER_NEAREST),
                       cv2.COLOR_GRAY2BGR)
    picks = []
    cv2.namedWindow('map')
    cv2.setMouseCallback(
        'map',
        lambda e, x, y, *_: e == cv2.EVENT_LBUTTONDOWN
        and picks.append((x / SCALE, y / SCALE)))

    while True:
        view = big.copy()
        for i, (px, py) in enumerate(picks):
            c = (int(px * SCALE), int(py * SCALE))
            x, y = to_world(px, py, h, res, origin)
            cv2.drawMarker(view, c, (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
            cv2.putText(view, f'{i + 1} ({x:.2f},{y:.2f})', (c[0] + 8, c[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        cv2.imshow('map', view)
        key = cv2.waitKey(20) & 0xFF
        if key == ord('q'):
            break
        if key == ord('r'):
            picks.clear()
    cv2.destroyAllWindows()

    if not picks:
        return
    print('\nhomography.py의 MAP_POINTS에 붙여넣으세요:\n')
    print('MAP_POINTS = [')
    for i, (px, py) in enumerate(picks):
        x, y = to_world(px, py, h, res, origin)
        print(f'    ({x:.3f}, {y:.3f}),   # {i + 1}번째')
    print(']')


def _self_check():
    """origin과 뒤집기가 맞는지 알려진 값으로 확인."""
    h, res, origin = 108, 0.05, [-2.07, -0.689, 0]
    # 좌하단 픽셀 = map 좌표의 최소 모서리 근처
    x, y = to_world(0, h - 1, h, res, origin)
    assert abs(x - (-2.045)) < 1e-9, x
    assert abs(y - (-0.664)) < 1e-9, y
    # 좌상단 픽셀 = y가 가장 큰 쪽
    _, ytop = to_world(0, 0, h, res, origin)
    assert abs(ytop - (origin[1] + (h - 0.5) * res)) < 1e-9, ytop
    assert ytop > y, '이미지 위쪽이 +y여야 한다'
    # 오른쪽으로 1px = +5cm
    x2, _ = to_world(1, h - 1, h, res, origin)
    assert abs((x2 - x) - res) < 1e-9
    print('map_point_picker self-check ok')


if __name__ == '__main__':
    if '--test' in sys.argv:
        _self_check()
    else:
        pick(sys.argv[1] if len(sys.argv) > 1 else MAP_YAML)
