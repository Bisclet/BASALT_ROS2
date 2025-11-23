from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    cam_calib_arg = DeclareLaunchArgument(
        'cam_calib',
        description='Path to camera calibration JSON file (REQUIRED)'
    )
    
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='',
        description='Path to VIO configuration JSON file'
    )

    basalt_vio_node = Node(
        package='basalt',
        executable='basalt_vio_ros2',
        name='basalt_vio_euroc',
        output='screen',
        parameters=[{
            'left_image_topic': '/cam0/image_raw',
            'right_image_topic': '/cam1/image_raw',
            'imu_topic': '/imu0',
            'odometry_topic': '/basalt/odom',
            'cam_calib': LaunchConfiguration('cam_calib'),
            'config_path': LaunchConfiguration('config_path'),
            'num_threads': 4,
        }]
    )

    return LaunchDescription([
        cam_calib_arg,
        config_path_arg,
        basalt_vio_node
    ])
