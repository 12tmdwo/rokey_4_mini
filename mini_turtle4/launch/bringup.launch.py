"""로케이터 2개 + goal 매니저를 한 번에 실행.

    ros2 launch mini_turtle4 bringup.launch.py namespace:=robot3

Nav2(localization + nav2)는 따로 띄우고 RViz에서 초기 위치를 먼저 찍을 것.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = ['webcam_locator_node', 'oakd_locator_node', 'goal_manager_node']

# tf2_ros의 TransformListener는 네임스페이스를 무시하고 절대 경로 '/tf'를 구독한다
# (transform_listener.py:85). 상대 경로로 리맵해야 '/robot4/tf'를 듣는다.
TF_REMAP = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    # 세 노드에 같은 네임스페이스를 걸어야 detections 토픽이 서로 맞는다.
    ns = LaunchConfiguration('namespace')
    return LaunchDescription(
        [DeclareLaunchArgument('namespace', default_value='',
                               description='로봇 네임스페이스 (예: robot4)')]
        + [Node(package='mini_turtle4', executable=n, name=n,
                namespace=ns, remappings=TF_REMAP,
                output='screen', emulate_tty=True)
           for n in NODES])
