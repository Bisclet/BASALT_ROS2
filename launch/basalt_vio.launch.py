from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    left_image_topic_arg = DeclareLaunchArgument(
        'left_image_topic',
        default_value='/cam0/image_raw',
        description='Left camera image topic'
    )
    
    right_image_topic_arg = DeclareLaunchArgument(
        'right_image_topic',
        default_value='/cam1/image_raw',
        description='Right camera image topic'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu0',
        description='IMU topic'
    )
    
    odometry_topic_arg = DeclareLaunchArgument(
        'odometry_topic',
        default_value='/basalt/odom',
        description='Output odometry topic'
    )
    
    cam_calib_arg = DeclareLaunchArgument(
        'cam_calib',
        default_value='',
        description='Path to camera calibration JSON file'
    )
    
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='',
        description='Path to VIO configuration JSON file'
    )

    # Create the node
    basalt_vio_node = Node(
        package='basalt',
        executable='basalt_vio_ros2',
        name='basalt_vio',
        output='screen',
        parameters=[{
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'cam_calib': LaunchConfiguration('cam_calib'),
            'config_path': LaunchConfiguration('config_path'),
        }]
    )

    return LaunchDescription([
        left_image_topic_arg,
        right_image_topic_arg,
        imu_topic_arg,
        odometry_topic_arg,
        cam_calib_arg,
        config_path_arg,
        basalt_vio_node
    ])
