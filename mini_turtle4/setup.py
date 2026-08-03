from glob import glob

from setuptools import find_packages, setup

package_name = 'mini_turtle4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='kny20031209@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cam_sub_node = mini_turtle4.cam_sub_node:main',
            'yolo_node = mini_turtle4.yolo_node:main',
            'depth_checker = mini_turtle4.depth_checker:main',
            'webcam_locator_node = mini_turtle4.webcam_locator_node:main',
            'oakd_locator_node = mini_turtle4.oakd_locator_node:main',
            'goal_manager_node = mini_turtle4.goal_manager_node:main',
            'dummy_obstacle_node = mini_turtle4.dummy_obstacle_node:main',
            'detection_marker_node = mini_turtle4.detection_marker_node:main',
        ],
    },
)
