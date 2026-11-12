"""Stereo depth pipeline for the rover (Project #2).

Starts the stereo camera driver with the rover's calibration file
(config/stereo_calib.yaml in this package) and the image_proc and
stereo_image_proc nodes that turn its images into depth, all in the
/stereo namespace:

  /stereo/left/image_raw, /stereo/right/image_raw   driver, 30 fps
  /stereo/left/image_rect, /stereo/right/image_rect image_proc (rectified, mono)
  /stereo/disparity                                 stereo_image_proc
  /stereo/points2                                   stereo_image_proc

The point cloud is in stereo_left_optical_frame, the rectified left
camera frame. The two cameras free-run, so the disparity and point cloud
nodes pair each left frame with the nearest right frame (approximate_sync).

The nodes are listed here instead of including stereo_image_proc's
launch file, so that:
  - they run in a container whose executor can be chosen
    (container_executable); stock stereo_image_proc runs all its nodes on
    one thread, the limit measured in Session 002;
  - the pairing queues can be longer (queue_size): with a queue of 5
    images (about 167 ms at 30 fps) the point cloud node lost every
    disparity that arrived later than that;
  - only the nodes depth needs run: with encoding mono8 the driver's
    images go straight to rectification; with bgr8 a debayer node makes
    the mono copy; a colour rectifier only runs when use_color is true.

Defaults chosen from the Session 002 measurements: mono8 (nothing here
uses colour, and the driver then needs far less CPU), the thread pool
container, and a packed point cloud without colour (about a third of
the size; Nav2 does not use colour). camera_fps sets both cameras' frame
rate, 15 by default: with the rest of the stack running the depth nodes
cannot keep up with 30 fps, and every frame they skip has still been
captured and rectified; at 15 fps the rover produced more point clouds,
with less delay and more valid points, than at 30.

Matching settings are stereo_image_proc's defaults except where an
argument below changes them. disparity_range 64 sees nothing closer
than about 0.45 m; 128 reaches about 0.23 m but loses depth in the
left 128 columns of the image.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('true', '1', 'yes')


def _processing(context):
    encoding = LaunchConfiguration('encoding').perform(context).strip()
    use_color = _flag(context, 'use_color')
    queue_size = int(LaunchConfiguration('queue_size').perform(context))
    nodes = []
    for side in ('left', 'right'):
        ns = '/stereo/' + side
        if encoding == 'mono8':
            mono = 'image_raw'
        else:
            mono = 'image_mono'
            nodes.append(ComposableNode(
                package='image_proc', plugin='image_proc::DebayerNode',
                name='debayer_node', namespace=ns))
        nodes.append(ComposableNode(
            package='image_proc', plugin='image_proc::RectifyNode',
            name='rectify_mono_node', namespace=ns,
            remappings=[('image', mono), ('image_rect', 'image_rect')]))
    cloud_image = 'left/image_rect'
    if use_color and encoding != 'mono8':
        cloud_image = 'left/image_rect_color'
        nodes.append(ComposableNode(
            package='image_proc', plugin='image_proc::RectifyNode',
            name='rectify_color_node', namespace='/stereo/left',
            remappings=[('image', 'image_color'), ('image_rect', 'image_rect_color')]))
    nodes.append(ComposableNode(
        package='stereo_image_proc', plugin='stereo_image_proc::DisparityNode',
        name='disparity_node', namespace='/stereo',
        parameters=[{
            'approximate_sync': True,
            'queue_size': queue_size,
            'disparity_range': int(LaunchConfiguration('disparity_range').perform(context)),
        }]))
    nodes.append(ComposableNode(
        package='stereo_image_proc', plugin='stereo_image_proc::PointCloudNode',
        name='point_cloud_node', namespace='/stereo',
        parameters=[{
            'approximate_sync': True,
            'queue_size': queue_size,
            'use_color': use_color,
            'avoid_point_cloud_padding': _flag(context, 'avoid_point_cloud_padding'),
        }],
        remappings=[('left/image_rect_color', cloud_image)]))
    return [ComposableNodeContainer(
        name='stereo_image_proc_container',
        namespace='/stereo',
        package='rclcpp_components',
        executable=LaunchConfiguration('container_executable').perform(context),
        composable_node_descriptions=nodes,
        output='screen',
    )]


def generate_launch_description():
    calibration = os.path.join(
        get_package_share_directory('robot_description'), 'config', 'stereo_calib.yaml')
    driver_launch = os.path.join(
        get_package_share_directory('stereo_camera_driver'), 'launch', 'stereo_camera.launch.py')

    # the driver's include sits in its own group, so the launch arguments it
    # declares (its 'namespace', for example) stay inside it
    driver = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
            launch_arguments={
                'calibration_file': calibration,
                'namespace': 'stereo',
                'encoding': LaunchConfiguration('encoding'),
                'source': LaunchConfiguration('source'),
                'fps': LaunchConfiguration('camera_fps'),
            }.items(),
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'disparity_range', default_value='64',
            description='Disparities searched, a multiple of 16 (64: nearest about 0.45 m; '
                        '128: about 0.23 m, no depth in the left 128 columns)'),
        DeclareLaunchArgument(
            'use_color', default_value='False',
            description='Colour in the point cloud'),
        DeclareLaunchArgument(
            'avoid_point_cloud_padding', default_value='True',
            description='Pack the point cloud without alignment padding'),
        DeclareLaunchArgument(
            'encoding', default_value='mono8',
            description='Camera image encoding: mono8 or bgr8'),
        DeclareLaunchArgument(
            'camera_fps', default_value='15',
            description='Frames per second per camera (up to 30)'),
        DeclareLaunchArgument(
            'source', default_value='argus',
            description='argus: the cameras; test: a GStreamer test pattern instead'),
        DeclareLaunchArgument(
            'container_executable', default_value='component_container_mt',
            description='component_container (one thread, as stock stereo_image_proc), '
                        'component_container_mt (a thread pool) or '
                        'component_container_isolated (one thread per node)'),
        DeclareLaunchArgument(
            'queue_size', default_value='15',
            description='Messages each pairing queue holds (15: about 0.5 s at 30 fps)'),
        driver,
        OpaqueFunction(function=_processing),
    ])
