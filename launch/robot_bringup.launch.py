"""localization + nav2 + RViz + lifecycle 활성화까지 한 번에.

    ros2 launch mini_turtle4 robot_bringup.launch.py

WiFi 환경에서는 lifecycle 전환 서비스가 자주 타임아웃 나서 노드가 unconfigured에
멈춘다. nav2_activate.sh를 같이 띄워서 멈춘 것을 자동으로 밀어 올린다.

뜬 뒤 할 일:
  1. RViz에서 2D Pose Estimate로 초기 위치 지정 (매번 필요 — AMCL이 기억 못 함)
  2. 다른 터미널에서  ros2 launch mini_turtle4 bringup.launch.py namespace:=robot4
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

NAMESPACE = '/robot4'
MAP = '/home/rokey/my_map.yaml'


def generate_launch_description():
    nav = get_package_share_directory('turtlebot4_navigation')
    viz = get_package_share_directory('turtlebot4_viz')
    mine = get_package_share_directory('mini_turtle4')
    ns = LaunchConfiguration('namespace')
    args = {'namespace': ns}.items()

    def include(pkg, name, extra=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', name])),
            launch_arguments=list(args) + (extra or []))

    # 스크립트가 노드를 기다렸다가 활성화하므로 지연 없이 같이 띄워도 된다.
    # 네임스페이스는 앞의 '/'를 뗀 형태를 쓴다 (ros2 lifecycle 경로 조립용).
    activate = ExecuteProcess(
        cmd=['bash', PathJoinSubstitution([mine, 'scripts', 'nav2_activate.sh']),
             NAMESPACE.lstrip('/')],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=NAMESPACE),
        DeclareLaunchArgument('map', default_value=MAP),
        include(nav, 'localization.launch.py',
                [('map', LaunchConfiguration('map'))]),
        include(nav, 'nav2.launch.py'),
        include(viz, 'view_robot.launch.py'),
        activate,
    ])
