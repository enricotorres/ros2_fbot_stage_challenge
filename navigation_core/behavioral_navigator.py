#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan


class BehavioralNavigatorNode(Node):
    def __init__(self):
        super().__init__("behavioral_navigator")
        self.get_logger().info("node %s initialized successfully." % self.get_name())

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.scan = None

        self.goal_x = 5.0
        self.goal_y = 4.0
        self.distance = 0.0
        self.angle_to_target = 0.0
        self.shortest_diff = 0.0

        self.state = "SEEK"
        self.hysteresis_count = 0
        self.hysteresis_threshold = 5
        self.angular_tolerance = 3.0
        self.goal_tolerance = 1.0
        self.path_safety_dist = 1.1
        self.safe_zone = 1.5
        self.angular_speed = 0.5
        self.linear_speed = 2.0

        self.odom_subscription = self.create_subscription(
            Odometry, "/ground_truth", self.ground_truth_callback, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, "base_scan", self.base_scan_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def ground_truth_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        W = msg.pose.pose.orientation.w
        Z = msg.pose.pose.orientation.z
        q = [0, 0, Z, W]
        r = Rotation.from_quat(q)
        yaw = r.as_euler("zyx", degrees=True)[0]
        self.current_yaw = self.normalize_angle(yaw)

    def base_scan_callback(self, msg: LaserScan):
        self.scan = msg

    def update_goal_state(self):
        delta_x = self.goal_x - self.current_x
        delta_y = self.goal_y - self.current_y
        self.distance = np.sqrt(delta_x**2 + delta_y**2)
        self.angle_to_target = self.normalize_angle(
            np.rad2deg(np.arctan2(delta_y, delta_x))
        )
        self.shortest_diff = self.normalize_angle(
            self.angle_to_target - self.current_yaw
        )

    def update_behavioral_state(self, is_obstacle_in_front, is_path_clear):

        if self.state == "SEEK":
            if is_obstacle_in_front and self.distance > self.safe_zone:
                self.state = "AVOID"
                self.get_logger().info(f"STATE: AVOID     (Obstacle ahead!)")

        elif self.state == "AVOID":
            if not is_obstacle_in_front:
                self.state = "BYPASS"
                self.get_logger().info(
                    f"STATE: BYPASS    (Front clear, moving straight)"
                )

        elif self.state == "BYPASS":
            if is_obstacle_in_front:
                self.state = "AVOID"
                self.get_logger().info(f"STATE: AVOID     (Hit obstacle, turning)")

            elif is_path_clear:
                self.hysteresis_count += 1
                if self.hysteresis_count >= self.hysteresis_threshold:
                    self.state = "SEEK"
                    self.hysteresis_count = 0
                    self.get_logger().info(f"STATE: SEEK      (Path is clear!)")
            else:
                self.hysteresis_count = 0

    def generate_velocity_command(self):
        cmd = Twist()

        if self.state == "SEEK":
            if abs(self.shortest_diff) > self.angular_tolerance:
                cmd.linear.x = 0.0
                cmd.angular.z = (
                    self.angular_speed
                    if self.shortest_diff > 0
                    else -self.angular_speed
                )
            else:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0

        elif self.state == "AVOID":
            cmd.linear.x = 0.0
            cmd.angular.z = self.get_free_direction()

        elif self.state == "BYPASS":
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        return cmd

    def timer_callback(self):

        if self.scan is None:
            return

        self.update_goal_state()

        if self.distance < self.goal_tolerance:
            self.get_logger().info("GOAL REACHED!")
            self.publisher_.publish(Twist())
            return

        is_obstacle_in_front = self.detect_obstacle(self.path_safety_dist)

        is_path_clear = False
        if self.distance < self.safe_zone:
            is_path_clear = True
        else:
            is_path_clear = self.is_goal_path_clear()

        self.update_behavioral_state(is_obstacle_in_front, is_path_clear)

        cmd = self.generate_velocity_command()

        self.publisher_.publish(cmd)

    def normalize_angle(self, angle):
        return ((angle + 180) % 360) - 180

    def detect_obstacle(self, distance_threshold, frontal_angle_deg=30.0):
        if self.scan is None:
            return False

        ranges = self.scan.ranges
        n = len(ranges)
        center_idx = n // 2

        half_window = int(np.deg2rad(frontal_angle_deg / 2) / self.scan.angle_increment)

        start_idx = max(0, center_idx - half_window)
        end_idx = min(n, center_idx + half_window)

        frontal_scan = ranges[start_idx:end_idx]

        valid = [
            r for r in frontal_scan if self.scan.range_min <= r <= self.scan.range_max
        ]

        return any(r < distance_threshold for r in valid) if valid else False

    def get_free_direction(self):
        if not self.scan:
            return self.angular_speed

        center_idx = int((0.0 - self.scan.angle_min) / self.scan.angle_increment)
        left_ranges = self.scan.ranges[center_idx:]
        right_ranges = self.scan.ranges[:center_idx]

        valid_left = [r for r in left_ranges if r >= self.scan.range_min]
        valid_right = [r for r in right_ranges if r >= self.scan.range_min]

        left_avg = np.mean(valid_left) if valid_left else 0.0
        right_avg = np.mean(valid_right) if valid_right else 0.0

        return self.angular_speed if left_avg > right_avg else -self.angular_speed

    def is_goal_path_clear(self, safety_dist=5.0):
        if self.scan is None:
            return False

        relative_angle_rad = np.deg2rad(self.shortest_diff)

        if not (self.scan.angle_min <= relative_angle_rad <= self.scan.angle_max):
            return False

        center_idx = int(
            (relative_angle_rad - self.scan.angle_min) / self.scan.angle_increment
        )
        window_deg = 10.0
        window_rad = np.deg2rad(window_deg)
        half_window_idx = int((window_rad / 2.0) / self.scan.angle_increment)

        start_idx = max(0, center_idx - half_window_idx)
        end_idx = min(len(self.scan.ranges) - 1, center_idx + half_window_idx)

        ranges_in_path = self.scan.ranges[start_idx:end_idx]

        valid_finite_ranges = [r for r in ranges_in_path if r >= self.scan.range_min]

        if not valid_finite_ranges:
            return True

        return not any(r < safety_dist for r in valid_finite_ranges)


def main(args=None):
    rclpy.init(args=args)
    combined_planner = BehavioralNavigatorNode()
    rclpy.spin(combined_planner)
    combined_planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
