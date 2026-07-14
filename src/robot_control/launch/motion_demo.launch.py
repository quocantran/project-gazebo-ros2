from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Khai báo các tham số đầu vào qua dòng lệnh khi chạy launch file
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='circle',
        description='Kịch bản chuyển động: forward, backward, left, right, stop, circle, square, figure_eight'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.25',
        description='Vận tốc dài tịnh tiến (m/s)'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.4',
        description='Vận tốc góc quay (rad/s)'
    )

    # Khởi chạy node điều khiển quỹ đạo
    motion_demo_node = Node(
        package='robot_control',
        executable='motion_demo',
        name='motion_demo_node',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed')
        }]
    )

    return LaunchDescription([
        mode_arg,
        linear_speed_arg,
        angular_speed_arg,
        motion_demo_node
    ])
