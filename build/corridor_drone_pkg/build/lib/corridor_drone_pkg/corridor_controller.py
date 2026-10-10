#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class CorridorController(Node):
    def __init__(self):
        super().__init__('corridor_controller')
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publish velocity commands continuously at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # State variables
        self.latest_scan = None
        self.forward_speed = 0.6     # m/s forward
        self.kp_lateral = 1.2        # Proportional gain for centering
        self.stop_threshold = 0.8    # Meters from front wall to stop
        self.finished = False

        self.get_logger().info('Autonomous Corridor Centering Controller Active')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def timer_callback(self):
        if self.finished:
            return

        cmd = Twist()

        # If no LiDAR data received yet, still drive forward slowly to initiate movement
        if self.latest_scan is None or len(self.latest_scan.ranges) == 0:
            cmd.linear.x = self.forward_speed
            self.cmd_pub.publish(cmd)
            return

        msg = self.latest_scan
        num_ranges = len(msg.ranges)

        # Index angles: 0 deg = Front, 90 deg = Left, 270 deg = Right
        front_idx = 0
        left_idx = num_ranges // 4
        right_idx = (3 * num_ranges) // 4

        front_dist = msg.ranges[front_idx]
        left_dist = msg.ranges[left_idx]
        right_dist = msg.ranges[right_idx]

        # Clean infinity / max range readings
        left_dist = left_dist if not math.isinf(left_dist) else msg.range_max
        right_dist = right_dist if not math.isinf(right_dist) else msg.range_max
        front_dist = front_dist if not math.isinf(front_dist) else msg.range_max

        # Stop condition at far end of corridor
        if front_dist <= self.stop_threshold:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'Reached corridor end (Front: {front_dist:.2f}m). Stopping!')
            self.finished = True
            return

        # Centering error (Left - Right)
        error = left_dist - right_dist

        cmd.linear.x = self.forward_speed
        cmd.linear.y = self.kp_lateral * error

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'Moving -> Left: {left_dist:.2f}m | Right: {right_dist:.2f}m | Front: {front_dist:.2f}m | Error: {error:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CorridorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()