from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Khai báo các tham số đầu vào qua dòng lệnh
    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='0.6',
        description='Khoảng cách tối thiểu phát hiện vật cản trước mặt (mét)'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.25',
        description='Vận tốc đi thẳng tịnh tiến (m/s)'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.5',
        description='Vận tốc góc quay khi tránh cản (rad/s)'
    )

    # Node tự hành tránh cản
    autonomous_node = Node(
        package='robot_control',
        executable='obstacle_avoidance',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[{
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed')
        }]
    )

    return LaunchDescription([
        obstacle_threshold_arg,
        linear_speed_arg,
        angular_speed_arg,
        autonomous_node
    ])
