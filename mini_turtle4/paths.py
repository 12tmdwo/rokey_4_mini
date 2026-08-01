"""프로젝트가 쓰는 파일 경로 모음.

저장소를 다른 곳에 받았으면 PKG 한 줄만 고치면 된다.

install/share가 아니라 소스 트리를 가리킨다 — homography.py가 새로 만든
homography.npy를 노드가 곧바로 읽어야 하는데, share를 쓰면 캘리브레이션할 때마다
colcon build를 해야 하고 빼먹으면 에러 없이 옛 좌표를 계속 쓰게 된다.
"""
from pathlib import Path

PKG = Path('/home/rokey/turtlebot4_ws/src/mini_turtle4')
RESOURCE = PKG / 'resource'

H_FILE = str(RESOURCE / 'homography.npy')       # 호모그래피 행렬
MAP_YAML = str(RESOURCE / 'my_map.yaml')        # SLAM 맵 (.pgm은 같은 폴더)
MODEL = '/home/rokey/turtlebot4_ws/best.pt'     # YOLO 가중치 (22MB, 밖에 둠)
