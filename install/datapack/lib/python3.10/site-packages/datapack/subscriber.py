# subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarActionNode(Node):
    def __init__(self):
        super().__init__("lidar_action_node")
        self.subscription = self.create_subscription(
            LaserScan,
            "/lidar_data_mock",
            self.scan_callback,
            10
        )
        self.get_logger().info("Lidar Action Node Started!")

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # front / left / right 영역 (publisher 범위와 맞춤)
        front = np.r_[ranges[350:360], ranges[0:10]]
        left  = ranges[80:100]
        right = ranges[260:280]

        front_dist = np.mean(front)
        left_dist  = np.mean(left)
        right_dist = np.mean(right)

        safe_dist = 0.5

        if front_dist < safe_dist:
            if left_dist > right_dist:
                action = "turn_left"
            elif right_dist > left_dist:
                action = "turn_right"
            else:
                action = "turn_left"  # 좌우 같으면 왼쪽
        else:
            action = "go_forward"

        self.get_logger().info(
            f"front:{front_dist:.2f}, left:{left_dist:.2f}, right:{right_dist:.2f} -> action:{action}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LidarActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
