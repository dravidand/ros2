#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import tf_transformations
import time

# Constants
MAX_LIN_VEL = 0.2
MAX_ANG_VEL = 0.5
OBSTACLE_THRESHOLD = 0.8
TOLERANCE = 0.1

class MultiWaypointNavigation(Node):
    def __init__(self):
        super().__init__('multi_waypoint_navigation')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Waypoints
        self.waypoints = [
            (7.15, 1.322), 
            (3.94, 3.01), 
            (1.52, 2.02),
            (-1.09, 1.0743),
            (-2.92, 1.05)]
        self.current_waypoint_idx = 0

        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.previous_x = None
        self.previous_y = None
        self.lidar_ranges = []

        # Distance and time tracking
        self.total_distance = 0.0
        self.start_time = time.time()  # Start time when script is triggered

        self.move_robot()

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Incremental distance calculation
        if self.previous_x is not None and self.previous_y is not None:
            dx = self.current_x - self.previous_x
            dy = self.current_y - self.previous_y
            self.total_distance += math.sqrt(dx**2 + dy**2)

        self.previous_x = self.current_x
        self.previous_y = self.current_y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(orientation_list)

    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges

    def move_robot(self):
        msg = Twist()

        while rclpy.ok():
            if not self.lidar_ranges:
                self.get_logger().warn("Waiting for Lidar data...")
                rclpy.spin_once(self)
                continue

            if self.current_waypoint_idx >= len(self.waypoints):
                elapsed_time = (time.time() - self.start_time) / 60  # Convert to minutes
                self.get_logger().info(f"All waypoints reached. Total distance: {self.total_distance:.2f} meters. Total time: {elapsed_time:.2f} minutes.")
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                break

            target_x, target_y = self.waypoints[self.current_waypoint_idx]
            distance = math.sqrt((target_x - self.current_x) ** 2 + (target_y - self.current_y) ** 2)

            # Print live distance and time
            elapsed_time = (time.time() - self.start_time) / 60  # Convert to minutes
            self.get_logger().info(f"Live distance traveled: {self.total_distance:.2f} meters. Elapsed time: {elapsed_time:.2f} minutes.")

            if distance < TOLERANCE:
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx + 1}. Waiting for 2 seconds...")
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                time.sleep(2)
                self.current_waypoint_idx += 1
                continue

            angle_to_target = math.atan2(target_y - self.current_y, target_x - self.current_x)
            angle_diff = self.normalize_angle(angle_to_target - self.current_yaw)

            if min(self.lidar_ranges[0:15]) > OBSTACLE_THRESHOLD and abs(angle_diff) < math.radians(15):
                msg.linear.x = MAX_LIN_VEL
                msg.angular.z = 0.0
                self.get_logger().info("Moving forward.")
            else:
                msg.linear.x = 0.0
                msg.angular.z = MAX_ANG_VEL if angle_diff > 0 else -MAX_ANG_VEL
                self.get_logger().info("Adjusting orientation.")

            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = MultiWaypointNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
