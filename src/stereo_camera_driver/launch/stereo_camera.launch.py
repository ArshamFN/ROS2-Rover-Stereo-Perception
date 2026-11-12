"""Start the stereo camera driver with a calibration file, logging the file's sha256."""

import hashlib
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _driver(context):
    path = os.path.expanduser(LaunchConfiguration('calibration_file').perform(context))
    with open(path, 'rb') as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    return [
        LogInfo(msg=f'stereo camera calibration file {path}, sha256 {digest}'),
        Node(
            package='stereo_camera_driver',
            executable='stereo_camera',
            name='stereo_camera',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'calibration_file': path,
                'encoding': LaunchConfiguration('encoding'),
                'sensor_mode': LaunchConfiguration('sensor_mode'),
                'source': LaunchConfiguration('source'),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_file',
            description='stereo calibration (OpenCV YAML from stereo_calibrate.py)'),
        DeclareLaunchArgument('encoding', default_value='bgr8', description='bgr8 or mono8'),
        DeclareLaunchArgument('sensor_mode', default_value='3', description='Argus sensor mode'),
        DeclareLaunchArgument(
            'source', default_value='argus', description='argus, or test for a test pattern'),
        DeclareLaunchArgument('namespace', default_value='stereo'),
        OpaqueFunction(function=_driver),
    ])
