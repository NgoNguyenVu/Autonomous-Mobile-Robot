import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'reject_bytes': False}],
        respawn=True
    )
    
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        respawn=True
    )

    tf2_web_republisher_node = Node(
        package='tf2_web_republisher',
        executable='tf2_web_republisher_node',
        name='tf2_web_republisher',
        respawn=True
    )

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera_node',
        parameters=[{
            'video_device': '/dev/video0',
            'framerate': 30.0,
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',  
            'io_method': 'mmap',     
            'camera_name': 'webcam',
            'brightness': 16,
            'contrast': 16
        }],
        remappings=[
            ('/image_raw', '/image_raw') 
        ],
        respawn=True
    )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': 8080,
            'server_threads': 4,
            'ros_threads': 2,
            'type': 'mjpeg',
            'default_transport': 'raw' 
        }],
        respawn=True
    )

    relay_script_path = os.path.join(
        os.path.expanduser('~'), 
        'ros2_ws', 
        'src', 
        'tf2_web_republisher', 
        'tf_relay.py'
    )

    start_tf_relay = ExecuteProcess(
        cmd=['python3', relay_script_path],
        output='screen',
        name='tf_relay_script'
    )

    return LaunchDescription([
        rosbridge_node,
        rosapi_node,
        tf2_web_republisher_node,
        camera_node,            
        web_video_server_node,
        start_tf_relay,
    ])
