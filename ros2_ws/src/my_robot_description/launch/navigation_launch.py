import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # --- LẤY ĐƯỜNG DẪN ---
    my_robot_description_pkg = get_package_share_directory('my_robot_description')
    sllidar_ros2_pkg = get_package_share_directory('sllidar_ros2')
    
    # --- LẤY ĐƯỜNG DẪN CHO NAV2 ---
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup') # Gói Nav2

    # --- ĐỊNH NGHĨA CÁC FILE CẤU HÌNH ---
    
    # 1. File bản đồ (map) bạn đã lưu
    map_file = os.path.join(
        my_robot_description_pkg,
        'maps',
        'my_saved_map.yaml' # <--- CHỈNH LẠI TÊN NẾU CẦN
    )

    # 2. File cấu hình Nav2 (mà chúng ta vừa sửa)
    nav2_params_file = os.path.join(
        my_robot_description_pkg,
        'config',
        'nav2_navigation_params.yaml' # <--- DÙNG FILE CONFIG NAV2
    )

    # --- BƯỚC 1: XỬ LÝ URDF (Đã sửa) ---
    xacro_file = os.path.join(my_robot_description_pkg, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # --- BƯỚC 2: CÁC NODE CƠ SỞ (Giữ nguyên) ---

    # Node giao tiếp với STM32
    node_stm32_bridge = Node(
        package='stm32_bridge',
        executable='stm32_bridge_node',
        name='stm32_bridge'
    )

    # Node tính toán Odometry (Đã sửa)
    node_odom_publisher = Node(
        package='mecanum_control',
        executable='odom_publisher',
        name='odom_publisher',
        parameters=[{'use_sim_time': False}]
    )

    # Node robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
        robot_description,
        {'use_sim_time': False},
        {'use_tf_static': False}]
    )

    # Khởi động node Lidar
    launch_sllidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_pkg, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={'frame_id': 'laser_frame', 'use_sim_time': 'false'}.items()
    )

    # --- BƯỚC 3: NODE MỚI - KHỞI ĐỘNG NAV2 ---
    # THAY THẾ HOÀN TOÀN 'slam_toolbox'
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',         # Chạy robot thật
            'map': map_file,                 # Cung cấp bản đồ
            'params_file': nav2_params_file, # Cung cấp file cấu hình Nav2
            'autostart': 'true'              # Tự động kích hoạt Nav2
        }.items(),
    )

    # Khởi động RViz (Nên dùng file config rviz riêng cho Nav2)
    #node_rviz = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    name='rviz2',
    #    output='screen',
    #     arguments=['-d', os.path.join(my_robot_description_pkg, 'rviz', 'navigation.rviz')]
    #)

    # --- BƯỚC 4: TRẢ VỀ DANH SÁCH CÁC NODE CẦN CHẠY ---
    return LaunchDescription([
        node_stm32_bridge,
        node_odom_publisher,
        node_robot_state_publisher,
        launch_sllidar,
        
        start_nav2_cmd, # <--- Đây là Nav2
        
        #node_rviz       
    ])
