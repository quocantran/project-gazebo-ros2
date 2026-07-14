import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node spawn robot từ topic '/robot_description' vào Gazebo Sim
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'robot_car',            # Tên robot trong Gazebo
            '-topic', 'robot_description',   # Topic chứa dữ liệu URDF từ robot_state_publisher
            '-x', '2.0',                     # Vị trí x khởi tạo (tránh va chạm kệ hàng khi spawn)
            '-y', '2.0',                     # Vị trí y khởi tạo
            '-z', '0.2',                     # Vị trí z khởi tạo
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'                      # Góc quay khởi tạo (Yaw)
        ]
    )

    return LaunchDescription([
        spawn_robot_node
    ])
