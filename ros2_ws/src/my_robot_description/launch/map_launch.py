import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro



def generate_launch_description():

    # --- LẤY ĐƯỜNG DẪN ---

    my_robot_description_pkg = get_package_share_directory('my_robot_description')

    sllidar_ros2_pkg = get_package_share_directory('sllidar_ros2')

    

    # --- LẤY ĐƯỜNG DẪN CHO SLAM ---

    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')





    # --- BƯỚC 1: XỬ LÝ FILE XACRO THÀNH URDF ---

    xacro_file = os.path.join(my_robot_description_pkg, 'urdf', 'robot.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)

    robot_description = {'robot_description': robot_description_config.toxml()}

    

    # --- TẠO FILE CONFIG CHO SLAM (Quan trọng) ---

    # Bạn phải tự tạo file này, copy từ file mẫu của slam_toolbox

    slam_params_file = os.path.join(

        my_robot_description_pkg, # Lưu file config trong package của bạn

        'config',

        'my_mapper_params.yaml' # Tên file config

    )





    # --- BƯỚC 2: ĐỊNH NGHĨA CÁC NODE ---



    # Node giao tiếp với STM32

    node_stm32_bridge = Node(

        package='stm32_bridge',

        executable='stm32_bridge_node',

        name='stm32_bridge'

    )



    # Node tính toán Odometry

    node_odom_publisher = Node(

        package='mecanum_control',

        executable='odom_publisher',

        name='odom_publisher',

    )



    # Node robot_state_publisher

    node_robot_state_publisher = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        output='screen',

        parameters=[robot_description] 

    )



    # Khởi động node Lidar

    launch_sllidar = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(sllidar_ros2_pkg, 'launch', 'sllidar_a1_launch.py')

        ),

        launch_arguments={'frame_id': 'laser_frame'}.items()

    )



    # --- NODE MỚI: KHỞI ĐỘNG SLAM_TOOLBOX ---

    start_slam_toolbox_cmd = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')

        ),

        launch_arguments={

            'use_sim_time': 'false', # Chạy robot thật nên là 'false'

            'slam_params_file': slam_params_file

        }.items(),

    )



    # Khởi động RViz (Bỏ comment để xem)

    node_rviz = Node(

        package='rviz2',

        executable='rviz2',

        name='rviz2',

        output='screen'

        # Bạn có thể thêm file config cho rviz:

        # arguments=['-d', os.path.join(my_robot_description_pkg, 'rviz', 'slam.rviz')]

    )



    # --- BƯỚC 3: TRẢ VỀ DANH SÁCH CÁC NODE CẦN CHẠY ---

    return LaunchDescription([

        # node_joint_state_publisher, # Sẽ nói ở dưới

        node_stm32_bridge,

        node_odom_publisher,

        node_robot_state_publisher,

        launch_sllidar,

        start_slam_toolbox_cmd, # <--- THÊM SLAM VÀO

        node_rviz                # <--- THÊM RVIZ VÀO

    ])