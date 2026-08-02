"""depth 카메라 좌표 계산. ROS 무관, 노드가 import해서 사용."""
import cv2
import numpy as np

PATCH = 5   # depth 샘플 반경 (픽셀). 한 점만 보면 0(무효)이 잘 나옴


def decode_depth(data, fmt):
    """compressedDepth CompressedImage.data -> 16UC1 depth 배열 (mm).

    compressedDepth는 PNG 앞에 12바이트 ConfigHeader(format enum + quantA +
    quantB)가 붙어 있고 cv_bridge가 이걸 못 벗긴다. 16UC1은 양자화 없이 raw
    PNG라 헤더만 잘라내면 그대로 mm 값이 나온다. (32FC1이면 quantA/B로 역변환이
    필요하지만 OAK-D stereo는 16UC1이다.) 헤더 없는 일반 compressed도 통과시킨다.
    """
    buf = np.frombuffer(data, np.uint8)
    if 'compressedDepth' in fmt:
        buf = buf[12:]
    return cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)


def to_depth_px(u, v, rgb_shape, depth_shape):
    """RGB 픽셀 -> depth 픽셀. 두 이미지 해상도가 달라서 필요."""
    # OAK-D의 RGB와 depth는 화각(HFOV 63.399도)이 같게 맞춰져 있어서, 해상도만
    # 비례로 맞추면 픽셀이 정확히 대응한다 (근사 아님). 실측(robot4 camera_info):
    #   rgb/image_raw   704x704  fx=569.95  cx=353.40   -> depth와 동일, 1:1 통과
    #   rgb/preview     320x320  fx=259.07  cx=160.64   -> depth 704와 배율 2.2
    #   depth stereo    704x704  fx=569.95  cx=353.40
    # 어느 RGB를 쓰든 rgb_shape/depth_shape만 맞으면 된다. 남는 오차는 int() 버림뿐.
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
    # robot4 실측 형상: preview 주점 -> depth 주점(353.40, 354.12)로 떨어져야 한다
    assert to_depth_px(160.64, 160.96, (320, 320), (704, 704)) == (353, 354)
    # image_raw는 depth와 동일 해상도라 1:1 통과 (소수 -> int 버림만)
    assert to_depth_px(353.40, 354.12, (704, 704), (704, 704)) == (353, 354)

    K = np.array([[500., 0., 320.], [0., 500., 200.], [0., 0., 1.]])
    assert deproject(320, 200, 2.0, K) == (0.0, 0.0, 2.0)   # 정중앙
    x, y, z = deproject(420, 200, 2.0, K)                   # 오른쪽 100px
    assert abs(x - 0.4) < 1e-9 and abs(y) < 1e-9 and z == 2.0

    d = np.zeros((704, 704), np.uint16)
    d[340:365, 340:365] = 1234                              # 1.234m 블록
    png = cv2.imencode('.png', d)[1].tobytes()
    out = decode_depth(b'\x00' * 12 + png, '16UC1; compressedDepth png')
    assert out.dtype == np.uint16 and depth_at(out, 352, 352) == 1.234  # 헤더 벗김
    assert depth_at(decode_depth(png, '16UC1; png'), 352, 352) == 1.234  # 헤더 없음
    print('depth_math self-check ok')


if __name__ == '__main__':
    _self_check()
