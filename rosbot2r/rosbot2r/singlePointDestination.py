#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import tf_transformations
import time

# Constants for robot and environment parameters
MAX_LIN_VEL = 0.2  # Max linear velocity
MAX_ANG_VEL = 0.5  # Max angular velocity
OBSTACLE_THRESHOLD = 0.6  # Distance threshold for obstacles
TOLERANCE = 0.1  # Distance tolerance to reach the goal
K_P_ANGULAR = 1.5  # Proportional gain for angular velocity

class ShortestPathNavigation(Node):
    def __init__(self):
        super().__init__('shortest_path_navigation')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Destination coordinates
        self.target_x = 4.5
        self.target_y = 3.2
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.lidar_ranges = []

        # Navigation loop
        self.move_robot()

    def odom_callback(self, msg):
        # Get position and orientation from Odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = tf_transformations.euler_from_quaternion(orientation_list)

    def lidar_callback(self, msg):
        # Store Lidar data
        self.lidar_ranges = msg.ranges

    def compute_heading_error(self):
        # Calculate the angle to the target
        target_angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        heading_error = target_angle - self.current_yaw
        
        # Normalize the angle to the range [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        return heading_error

    def move_robot(self):
        msg = Twist()

        while rclpy.ok():
            # Wait for Lidar data
            if not self.lidar_ranges:
                self.get_logger().warn("Lidar data not available yet!")
                rclpy.spin_once(self)
                continue

            # Distance to the target
            distance_to_target = math.sqrt((self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2)

            # Check if the robot has reached the target
            if distance_to_target < TOLERANCE:
                self.get_logger().info("Destination reached!")
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                break

            # Obstacle avoidance
            forward_space = min(min(self.lidar_ranges[0:15] + self.lidar_ranges[-15:]), 10.0)  # Forward region
            heading_error = self.compute_heading_error()

            if forward_space > OBSTACLE_THRESHOLD:
                # Move forward with angular correction
                msg.linear.x = MAX_LIN_VEL
                msg.angular.z = K_P_ANGULAR * heading_error
                self.get_logger().info(f"Moving towards target: Distance: {distance_to_target:.2f}, Heading Error: {heading_error:.2f}")
            else:
                # Obstacle detected; decide whether to turn left or right
                left_space = min(self.lidar_ranges[45:90], default=10.0)
                right_space = min(self.lidar_ranges[270:315], default=10.0)
                if left_space > right_space:
                    msg.linear.x = 0.0
                    msg.angular.z = MAX_ANG_VEL
                    self.get_logger().info("Obstacle ahead, turning left.")
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = -MAX_ANG_VEL
                    self.get_logger().info("Obstacle ahead, turning right.")

            # Publish the command
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    shortest_path_navigation = ShortestPathNavigation()
    rclpy.spin(shortest_path_navigation)
    shortest_path_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
