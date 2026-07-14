from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node teleop keyboard điều khiển thủ công
    teleop_node = Node(
        package='robot_control',
        executable='teleop_keyboard',
        name='teleop_keyboard_node',
        output='screen',
        emulate_tty=True  # Quan trọng: Để terminal in màu sắc và bắt sự kiện bàn phím mượt mà
    )

    return LaunchDescription([
        teleop_node
    ])
