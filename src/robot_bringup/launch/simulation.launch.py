import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Đọc và phân giải thư mục share của các package liên quan
    description_share = get_package_share_directory('robot_description')
    gazebo_share = get_package_share_directory('robot_gazebo')
    bringup_share = get_package_share_directory('robot_bringup')

    # Khai báo các đối số dòng lệnh (Launch Arguments)
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Có khởi chạy RViz để trực quan hóa dữ liệu không?'
    )

    # 2. Xử lý mô hình Xacro sang URDF để nạp vào robot_state_publisher
    xacro_file = os.path.join(description_share, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    # Node: robot_state_publisher (Tính toán hệ trục tọa độ TF tĩnh từ file mô hình)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True
        }]
    )

    # 3. Cấu hình đường dẫn thế giới Warehouse
    world_file = os.path.join(gazebo_share, 'worlds', 'warehouse.sdf')

    # Thiết lập biến môi trường để Gazebo tìm thấy meshes cục bộ của robot
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join(description_share, '..')

    # 3a. Gazebo Server (headless): physics + sensors
    #     Ép software rendering CHỈ cho tiến trình server
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_file],
        output='screen',
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
    )

    # 3b. Gazebo GUI: hiển thị 3D bằng GPU ảo VMware (Ogre 1)
    #     Khởi chạy GUI sau server 2 giây để đảm bảo server sẵn sàng
    gazebo_gui = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'sim', '-g', '--render-engine', 'ogre'],
                output='screen',
            )
        ]
    )

    # 4. Spawner Node: Spawn robot từ robot_description topic vào Gazebo Sim
    spawn_robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'spawn_robot.launch.py')
        )
    )

    # 5. Node: ros_gz_bridge nạp cấu hình ánh xạ từ file YAML
    bridge_config = os.path.join(gazebo_share, 'config', 'bridge.yaml')
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True
        }]
    )

    # 6. Khởi chạy RViz2 trực quan hóa (nếu được kích hoạt rviz:=true)
    rviz_config = os.path.join(description_share, 'rviz', 'view_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_gui,
        spawn_robot_node,
        ros_gz_bridge_node,
        rviz_node
    ])
