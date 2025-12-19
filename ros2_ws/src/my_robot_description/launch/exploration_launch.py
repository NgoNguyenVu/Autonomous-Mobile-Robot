import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # --- LẤY ĐƯORNG DẪN ---
    my_robot_description_pkg = get_package_share_directory('my_robot_description')
    sllidar_ros2_pkg = get_package_share_directory('sllidar_ros2')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    # Vẫn cần path của slam_toolbox để lấy file config
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox') 

    # --- ĐỊNH NGHĨA CÁC FILE CẤU HÌNH ---

    # 1. File cấu hình Nav2 (Dùng file cũ của bạn)
    nav2_params_file = os.path.join(
        my_robot_description_pkg,
        'config',
        'nav2_exploration_params.yaml'
    )

    # 2. File cấu hình SLAM (Dùng file mặc định)
    slam_params_file = os.path.join(
        my_robot_description_pkg,      # <-- Sửa thành package của bạn
        'config',
        'my_slam_config.yaml'          # <-- Sửa thành tên file mới
    )

    # --- BƯỚC 1: XỬ LÝ URDF (Giữ nguyên) ---
    xacro_file = os.path.join(my_robot_description_pkg, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- BƯỚC 2: CÁC NODE CƠ SỞ (Giữ nguyên) ---
    node_stm32_bridge = Node(
        package='stm32_bridge',
        executable='stm32_bridge_node',
        name='stm32_bridge'
    )
    node_odom_publisher = Node(
        package='mecanum_control',
        executable='odom_publisher',
        name='odom_publisher',
        parameters=[{'use_sim_time': False}]
    )
# BẢN SỬA LỖI
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': False},
            {'use_tf_static': False}  # <-- Thêm dòng này
        ]
    )
    launch_sllidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_pkg, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={'frame_id': 'laser_frame', 'use_sim_time': 'false'}.items()
    )

    # --- BƯỚC 3: KHỞI ĐỘNG NAV2 + SLAM + EXPLORER (Đã sửa) ---

    # 3.1: KHỞI ĐỘNG NAV2 Ở CHẾ ĐỘ SLAM
    # Dùng 'slam_launch.py' sẽ tự động khởi động cả Nav2 và SLAM
    # và KHÔNG khởi động AMCL.
# 3.1: KHỞI ĐỘNG SLAM (CHỈ SLAM)
# Chúng ta dùng "online_sync_launch.py" của slam_toolbox
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_sync_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_params_file # Chỉ cần file config của SLAM
        }.items()
    )

    # 3.2: KHỞI ĐỘNG NAV2 (CHỈ NAV2)
    # Chúng ta dùng "navigation_launch.py"
    # Nó sẽ tự động dùng "lifecycle_manager" trong file YAML
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file, # File config Nav2 (có 'lifecycle_manager:')
            'autostart': 'true'
            # Chúng ta KHÔNG DÙNG AMCL, vì SLAM đã cung cấp map->odom
        }.items()
    )

    # 3.2: KHỞI ĐỘNG NODE EXPLORER ("CÁI NÃO")
    start_explorer_node_cmd = Node(
        package='custom_explorer',
        executable='explorer', # <-- Sửa từ 'explorer.py' thành 'explorer'
        name='explorer',
        output='screen'
    )

    # --- BƯỚC 4: RViz (Giữ nguyên) ---
    # node_rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    # )

    # --- BƯỚC 5: TRẢ VỀ DANH SÁCH ---
    return LaunchDescription([
        node_stm32_bridge,
        node_odom_publisher,
        node_robot_state_publisher,
        launch_sllidar,

    # Khởi động SLAM
        start_slam_toolbox_cmd,

        # Khởi động Nav2
        start_nav2_cmd,

        # Khởi động Explorer
        start_explorer_node_cmd,

        #node_rviz 
    ])