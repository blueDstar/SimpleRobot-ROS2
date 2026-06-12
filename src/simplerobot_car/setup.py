from setuptools import setup
import os
from glob import glob

package_name = 'simplerobot_car'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blueDstar',
    maintainer_email='datnvan.021504@gmail.com',
    description='SimpleRobot lane following and control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_lidar_follower_node = simplerobot_car.lane_lidar_follower_node:main',
            'line_camera_target_node = simplerobot_car.line_camera_target_node:main',
            'run_report_node = simplerobot_car.run_report_node:main',
            'cmdvel_odom_node = simplerobot_car.cmdvel_odom_node:main',
            'path_visualizer_node = simplerobot_car.path_visualizer_node:main',
            'lidar_avoidance_node = simplerobot_car.lidar_avoidance_node:main',
            'lane_parser_node = simplerobot_car.lane_parser_node:main',
            'lane_backstepping_pd = simplerobot_car.lane_backstepping_pd:main',
            'camera_record_node = simplerobot_car.camera_record_node:main',
            'drive_record_node = simplerobot_car.drive_record_node:main',
            'drive_capture_node = simplerobot_car.drive_capture_node:main',
        ],
    },
)
