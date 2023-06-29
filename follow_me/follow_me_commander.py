#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan

# from simple_pid import PID

HUMAN_LOCATION_TOPIC = "/human_location"
JOY_TOPIC = "/joy_teleop/joy"
CMD_VEL_TOPIC = "/husky_velocity_controller/cmd_vel_unstamped"
SCAN_TOPIC = "/scan"


class FollowMeCommander(Node):
    def __init__(self):
        super().__init__("follow_me_commander")

        self.declare_parameter("defined_distance", 0.8)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("follow_me_timeout", 0.5)
        self.declare_parameter("rel_under_min_dist", 0.05)
        self.max_velocity = self.get_parameter("max_velocity").value
        self.rel_under_min_dist = self.get_parameter("rel_under_min_dist").value

        self.fov = np.array(
            [20, 340]
        )  # Field of view in which obstacles are considered

        # initialize the dead-man timer
        self.following = False
        self.freeze = False
        self.follow_me_timer = self.create_timer(
            self.get_parameter("follow_me_timeout").value, self.stop_following
        )
        self.follow_me_timer.cancel()

        # initialize the subscribers and publishers
        self.create_subscription(Point, HUMAN_LOCATION_TOPIC, self.point_callback, 10)
        self.create_subscription(Joy, JOY_TOPIC, self.joy_callback, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

    def stop_following(self):
        self.following = False
        self.publish_velocity(0.0, 0.0)

    def point_callback(self, point_msg: Point):
        """Subscribes to the human location topic, calculates the desired velocities before publishing them."""
        if not self.following:
            return
        distance = (point_msg.x**2 + point_msg.y**2) ** 0.5
        defined_distance = self.get_parameter("defined_distance").value
        if distance < defined_distance:
            self.publish_velocity(0.0, 0.0)  # stop
        else:
            # calculate the desired velocity and direction
            velocity = self.max_velocity * (distance - defined_distance) / distance
            direction = point_msg.y / distance
            if not self.freeze:
                self.publish_velocity(float(velocity), float(direction))

    def scan_callback(self, scan_msg: LaserScan):
        """Subscribes to the laser scan topic and stops the robot if an obstacle is detected."""

        # Calculate corresponding indices and extract relevant scan data
        readings_per_radian = len(scan_msg.ranges) / (
            scan_msg.angle_max - scan_msg.angle_min
        )
        fov_rad = self.fov * np.pi / 180.0
        fov_indices = ((fov_rad - scan_msg.angle_min) * readings_per_radian).astype(int)
        scan_segment = scan_msg.ranges[fov_indices[0] : fov_indices[1]]

        if not scan_segment:
            return

        # Calculate the percentage of the scan segment that is below the minimum scan distance
        min_scan_distance = max(
            abs(self.max_velocity) * self.get_parameter("follow_me_timeout").value,
            self.get_parameter("defined_distance").value,
        )

        under_threshold = np.sum(np.array(scan_segment) < min_scan_distance) / len(
            scan_segment
        )

        if under_threshold > self.rel_under_min_dist:
            if self.following:
                self.stop_following()
                self.get_logger().info("Obstacle detected. Stopping.")
            self.freeze = True
        else:
            self.freeze = False

    def publish_velocity(self, linear_velocity: float, angular_velocity: float):
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist)

    def joy_callback(self, joy_msg: Joy):
        """Subscribes to the joystick topic and starts/stops the following behavior.
        Button X ~ 0: Stop immediately
        Button B ~ 2: Start following / reset the dead-man timer
        Button A ~ 1: Decrease max_velocity
        Button Y ~ 3: Increase max_velocity
        """
        x, a, b, y = joy_msg.buttons[:4]

        if x == 1:
            self.stop_following()
        elif b == 1:
            self.following = True
            self.follow_me_timer.reset()
        elif a == 1:
            self.max_velocity -= 0.05
            self.get_logger().info(f"Decreased max_velocity to {self.max_velocity}.")
        elif y == 1:
            self.max_velocity += 0.05
            self.get_logger().info(f"Increased max_velocity to {self.max_velocity}.")


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeCommander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
