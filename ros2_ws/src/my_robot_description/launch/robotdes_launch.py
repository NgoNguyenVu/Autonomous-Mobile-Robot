import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- 1. LẤY ĐƯỜNG DẪN GÓI ---
    pkg_name = 'my_robot_description'
    pkg_share = get_package_share_directory(pkg_name)

    # --- 2. XỬ LÝ FILE XACRO THÀNH URDF ---
    # Đảm bảo đường dẫn file xacro đúng với cấu trúc thư mục của bạn
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    
    # Process file xacro
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- 3. ĐỊNH NGHĨA CÁC NODE ---

    # A. Robot State Publisher
    # Node này chịu trách nhiệm công bố cây TF dựa trên file URDF
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # B. Joint State Publisher GUI
    # QUAN TRỌNG: Vì không chạy phần cứng (STM32), ta cần node này 
    # để giả lập trạng thái các khớp (bánh xe). Nó sẽ hiện 1 cửa sổ nhỏ
    # để bạn kéo thanh trượt kiểm tra các khớp.
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # C. Rviz2
    # Mở giao diện 3D
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        # Nếu bạn đã lưu cấu hình rviz trước đó thì bỏ comment dòng dưới:
        # arguments=['-d', os.path.join(pkg_share, 'rviz', 'view_robot.rviz')]
    )

    # --- 4. TRẢ VỀ ---
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
