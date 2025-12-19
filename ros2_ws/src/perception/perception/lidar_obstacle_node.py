import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_node')

        self.get_logger().info('Node lidar_obstacle_node khởi tạo thành công.')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.get_logger().info('Subscription tới topic /scan đã tạo.')

    def scan_callback(self, msg):
        self.get_logger().info(f"Callback /scan được gọi, nhận {len(msg.ranges)} giá trị.")

        # Lọc khoảng cách không hợp lệ (inf, nan)
        valid_ranges = [r for r in msg.ranges if r > 0.05 and r < 5.0]
        if not valid_ranges:
            self.get_logger().warn('Không có giá trị khoảng cách hợp lệ trong scan.')
            return

        min_distance = min(valid_ranges)

        self.get_logger().info(f'Vật cản gần nhất: {min_distance:.2f} m')

        if min_distance < 0.5:
            self.get_logger().warn('⚠️ Vật cản quá gần! DỪNG LẠI!')
            # Ở đây bạn có thể publish lệnh stop nếu muốn

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
