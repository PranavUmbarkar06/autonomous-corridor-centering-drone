#!/usr/bin/env python3
from collections import deque
import json
import math
import statistics
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class TelemetryWindow:
    """Keep numerical metrics and summarize their recent 30-second window."""

    def __init__(self, window_seconds=30.0):
        self.window_seconds = window_seconds
        self.samples = deque()

    def add_sample(self, timestamp, metrics):
        self.samples.append((timestamp, metrics))
        cutoff = timestamp - self.window_seconds
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    @staticmethod
    def _percentile(sorted_values, fraction):
        position = (len(sorted_values) - 1) * fraction
        lower = math.floor(position)
        upper = math.ceil(position)
        weight = position - lower
        return sorted_values[lower] * (1 - weight) + sorted_values[upper] * weight

    def summary(self):
        if not self.samples:
            return {}

        metric_names = self.samples[0][1].keys()
        result = {}
        for name in metric_names:
            values = sorted(sample[name] for _, sample in self.samples)
            if not values:
                continue
            result[name] = {
                'mean': statistics.mean(values),
                'std': statistics.pstdev(values),
                'min': values[0],
                'max': values[-1],
                'iqr': self._percentile(values, 0.75) - self._percentile(values, 0.25),
            }
        result['sample_count'] = len(self.samples)
        return result


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
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.latest_scan = None

        # Flight State Machine
        self.state = 'TAKEOFF'
        self.takeoff_start_time = time.monotonic()
        self.takeoff_duration = 3.0   # Ascend for 3 seconds
        self.landing_start_time = 0.0

        self.forward_speed = 0.5
        self.kp_lateral = 1.0
        self.stop_threshold = 1.0
        self.open_end_confirm_scans = 5
        self.open_end_count = 0
        self.finished = False

        self.telemetry = TelemetryWindow(window_seconds=30.0)
        self.last_telemetry_report = time.monotonic()

        self.get_logger().info('Flight-State Corridor Controller Active')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    @staticmethod
    def get_range_in_sector(msg: LaserScan, target_angle: float, half_width: float):
        """Return the median finite obstacle range in an angular sector, or None."""
        ranges = []
        for index, value in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            angle_delta = math.atan2(
                math.sin(angle - target_angle), math.cos(angle - target_angle)
            )
            if abs(angle_delta) <= half_width:
                if math.isfinite(value) and msg.range_min <= value < msg.range_max:
                    ranges.append(value)
        return statistics.median(ranges) if ranges else None

    def _record_telemetry(self, started_at, tracking_error, forward_speed, vertical_speed):
        now = time.monotonic()
        self.telemetry.add_sample(now, {
            'lateral_tracking_error': tracking_error,
            'forward_speed': forward_speed,
            'vertical_speed': vertical_speed,
            'loop_latency_seconds': time.perf_counter() - started_at,
        })
        if now - self.last_telemetry_report >= 30.0:
            self.get_logger().info(
                f'TELEMETRY {json.dumps(self.telemetry.summary(), sort_keys=True)}'
            )
            self.last_telemetry_report = now

    def timer_callback(self):
        if self.finished:
            return

        started_at = time.perf_counter()
        current_time = time.monotonic()
        cmd = Twist()

        # --- STATE 1: TAKEOFF ---
        if self.state == 'TAKEOFF':
            elapsed = current_time - self.takeoff_start_time
            if elapsed < self.takeoff_duration:
                cmd.linear.z = 0.4  # Vertical ascension command
                self.cmd_pub.publish(cmd)
                self._record_telemetry(started_at, 0.0, 0.0, cmd.linear.z)
                self.get_logger().info(f'State: TAKEOFF | Ascending... ({elapsed:.1f}s)')
                return
            else:
                self.state = 'CRUISE'
                self.get_logger().info(
                    'Takeoff complete. Entering CRUISE corridor centering mode.'
                )

        # --- STATE 3: LANDING ---
        if self.state == 'LANDING':
            landing_elapsed = current_time - self.landing_start_time
            if landing_elapsed < 2.5:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.linear.z = -0.4  # Smooth descent command
                self.cmd_pub.publish(cmd)
                self._record_telemetry(started_at, 0.0, 0.0, cmd.linear.z)
                self.get_logger().info(f'State: LANDING | Descending... ({landing_elapsed:.1f}s)')
            else:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0
                self.cmd_pub.publish(cmd)
                self.get_logger().info('Mission Complete: Drone Landed Safely.')
                self.finished = True
                self._record_telemetry(started_at, 0.0, 0.0, 0.0)
                self.timer.cancel()
            return

        # --- STATE 2: CRUISE / CORRIDOR CENTERING ---
        if self.latest_scan is None or len(self.latest_scan.ranges) == 0:
            self.cmd_pub.publish(cmd)
            self._record_telemetry(started_at, 0.0, 0.0, 0.0)
            return

        msg = self.latest_scan
        sector_half_width = math.radians(5.0)
        front_dist = self.get_range_in_sector(msg, 0.0, sector_half_width)
        left_dist = self.get_range_in_sector(msg, math.pi / 2, sector_half_width)
        right_dist = self.get_range_in_sector(msg, -math.pi / 2, sector_half_width)
        front_left_dist = self.get_range_in_sector(msg, math.pi / 4, sector_half_width)
        front_right_dist = self.get_range_in_sector(msg, -math.pi / 4, sector_half_width)

        if front_dist is not None and front_dist <= self.stop_threshold:
            self.state = 'LANDING'
            self.landing_start_time = time.monotonic()
            self.get_logger().info(
                f'Reached end of corridor (Front: {front_dist:.2f}m). Initiating LANDING sequence!'
            )
            self._record_telemetry(started_at, 0.0, 0.0, 0.0)
            return

        # Check for open corridor end
        expected_diagonal_range = None
        if left_dist is not None and right_dist is not None:
            expected_diagonal_range = max(left_dist, right_dist) * math.sqrt(2.0)
        open_end_visible = (
            front_dist is None
            and expected_diagonal_range is not None
            and expected_diagonal_range < msg.range_max
            and front_left_dist is None
            and front_right_dist is None
        )
        self.open_end_count = self.open_end_count + 1 if open_end_visible else 0
        if self.open_end_count >= self.open_end_confirm_scans:
            self.state = 'LANDING'
            self.landing_start_time = time.monotonic()
            self.get_logger().warning('Open corridor end detected from /scan. Initiating LANDING.')
            self._record_telemetry(started_at, 0.0, 0.0, 0.0)
            return

        if left_dist is None or right_dist is None:
            self.cmd_pub.publish(cmd)
            self._record_telemetry(started_at, 0.0, 0.0, 0.0)
            return

        error = left_dist - right_dist
        cmd.linear.x = self.forward_speed
        cmd.linear.y = self.kp_lateral * error
        cmd.linear.z = 0.0  # Maintain height during cruise

        self.cmd_pub.publish(cmd)
        self._record_telemetry(started_at, error, cmd.linear.x, cmd.linear.z)


def main(args=None):
    rclpy.init(args=args)
    node = CorridorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
