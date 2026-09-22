import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_profile', default_value='640x480x30'),
        DeclareLaunchArgument('gripper_depth_camera_no', default_value='_412622271789'),
        # Orbbec 336L parameters
        DeclareLaunchArgument('orbbec_color_width', default_value='640'),
        DeclareLaunchArgument('orbbec_color_height', default_value='480'),
        DeclareLaunchArgument('orbbec_color_fps', default_value='30'),
        DeclareLaunchArgument('orbbec_depth_width', default_value='640'),
        DeclareLaunchArgument('orbbec_depth_height', default_value='480'),
        DeclareLaunchArgument('orbbec_depth_fps', default_value='30'),
        DeclareLaunchArgument('orbbec_camera_name', default_value='orbbec'),
    ]
    camera_fps = LaunchConfiguration('camera_fps')
    camera_height = LaunchConfiguration('camera_height')
    camera_width = LaunchConfiguration('camera_width')
    camera_profile = LaunchConfiguration('camera_profile')
    gripper_depth_camera_no = LaunchConfiguration('gripper_depth_camera_no')
    orbbec_color_width = LaunchConfiguration('orbbec_color_width')
    orbbec_color_height = LaunchConfiguration('orbbec_color_height')
    orbbec_color_fps = LaunchConfiguration('orbbec_color_fps')
    orbbec_depth_width = LaunchConfiguration('orbbec_depth_width')
    orbbec_depth_height = LaunchConfiguration('orbbec_depth_height')
    orbbec_depth_fps = LaunchConfiguration('orbbec_depth_fps')
    orbbec_camera_name = LaunchConfiguration('orbbec_camera_name')

    # Realsense 深度相机 (Gripper 上). 必须绑定序列号，否则多台
    # Realsense 同时在线时驱动会自动选择第一台，容易采到 pika_sense 视角。
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={'serial_no': gripper_depth_camera_no,
                          'camera_namespace': 'gripper',
                          'camera_name': 'camera',
                          'rgb_camera.color_profile': camera_profile,
                          'depth_module.color_profile': camera_profile,
                          'depth_module.depth_profile': camera_profile,
                          'depth_module.infra_profile': camera_profile}.items()
    )

    # Orbbec Gemini 336L 深度相机 (第三视角)
    orbbec_camera = Node(
        package='sensor_tools',
        executable='orbbec_camera.py',
        name='orbbec_camera',
        parameters=[{'color_width': orbbec_color_width,
                     'color_height': orbbec_color_height,
                     'color_fps': orbbec_color_fps,
                     'depth_width': orbbec_depth_width,
                     'depth_height': orbbec_depth_height,
                     'depth_fps': orbbec_depth_fps,
                     'enable_color': True,
                     'enable_depth': False,
                     'align_depth_to_color': False,
                     'camera_name': orbbec_camera_name,
                     'camera_frame_id': 'orbbec/camera_link'}],
        respawn=True,
        output='screen',

    )

    return LaunchDescription(declared_arguments + [
        depth_camera_launch,
        orbbec_camera,
    ])
