"""로케이터 2개 + goal 매니저 + dummy 장애물을 한 번에 실행.

    ros2 launch mini_turtle4 bringup.launch.py namespace:=robot4

웹캠이 다른 PC(멀리)에 붙어 있으면, 그 PC에서 webcam_locator만 따로 띄우고 여기선
webcam:=false 로 웹캠 노드를 빼서 나머지 3개만 실행한다:

    # 메인 PC
    ros2 launch mini_turtle4 bringup.launch.py namespace:=robot4 webcam:=false
    # 웹캠 PC
    ros2 run mini_turtle4 webcam_locator_node --ros-args -r __ns:=/robot4

Nav2(localization + nav2)는 따로 띄우고 RViz에서 초기 위치를 먼저 찍을 것.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WEBCAM_NODE = 'webcam_locator_node'   # 유일하게 물리 웹캠을 여는 노드
OTHER_NODES = ['oakd_locator_node', 'goal_manager_node', 'dummy_obstacle_node']

# tf2_ros의 TransformListener는 네임스페이스를 무시하고 절대 경로 '/tf'를 구독한다
# (transform_listener.py:85). 상대 경로로 리맵해야 '/robot4/tf'를 듣는다.
TF_REMAP = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    # 같은 네임스페이스를 걸어야 detections 토픽이 서로 맞는다.
    ns = LaunchConfiguration('namespace')

    def node(name, condition=None):
        return Node(package='mini_turtle4', executable=name, name=name,
                    namespace=ns, remappings=TF_REMAP,
                    output='screen', emulate_tty=True, condition=condition)

    return LaunchDescription(
        [DeclareLaunchArgument('namespace', default_value='',
                               description='로봇 네임스페이스 (예: robot4)'),
         DeclareLaunchArgument('webcam', default_value='true',
                               description='웹캠 로케이터도 여기서 띄울지 (웹캠이 다른 PC면 false)'),
         node(WEBCAM_NODE, IfCondition(LaunchConfiguration('webcam')))]
        + [node(n) for n in OTHER_NODES])
