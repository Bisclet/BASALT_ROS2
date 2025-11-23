from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        description='Path to ROS2 bag file'
    )
    
    cam_calib_arg = DeclareLaunchArgument(
        'cam_calib',
        description='Path to camera calibration JSON file'
    )
    
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='',
        description='Path to VIO configuration JSON file'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier'
    )

    basalt_vio_node = Node(
        package='basalt',
        executable='basalt_vio_ros2',
        name='basalt_vio',
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

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', 
             LaunchConfiguration('bag_file'),
             '--rate', LaunchConfiguration('rate')],
        output='screen'
    )

    return LaunchDescription([
        bag_file_arg,
        cam_calib_arg,
        config_path_arg,
        rate_arg,
        basalt_vio_node,
        bag_play
    ])
