#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import Int64MultiArray
import math

class StepperDiffDrive(Node):
    def __init__(self):
        super().__init__('stepper_diff_drive')

        # Parameters
        self.wheel_track = 0.3876   # units: m , to be changed
        self.EncoderCountsPerWheel = 396 # to be changed
        self.wheel_radius = 0.0575  # units: m , to be changed

        # Odometry variables
        self._PreviousEncoderCounts = [0, 0, 0, 0]
        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        self.DistancePerCount = (math.pi * 2 * self.wheel_radius) / (self.EncoderCountsPerWheel * 4)

        # Spot Turn and Crab Crawl flags
        self.spot_turn_active = False
        self.crab_crawl_active = False

        # Robot state variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v_left = [0.0, 0.0]  # left wheel speeds (two wheels on the left side)
        self.v_right = [0.0, 0.0]  # right wheel speeds (two wheels on the right side)
        self.vth = 0.0  # angular velocity of the robot
        self.delta_left = [0.0, 0.0]  # no of ticks in left encoders since the last update
        self.delta_right = [0.0, 0.0]  # no of ticks in right encoders since the last update
        self.dt = 0.0
        self.delta_distance = 0.0  # distance moved by robot since last update
        self.delta_th = 0.0  # corresponding change in heading
        self.delta_x = 0.0  # corresponding change in x direction
        self.delta_y = 0.0  # corresponding change in y direction

        # Subscribers and Publishers
        self.motor_encoder_sub = self.create_subscription(Int64MultiArray, '/motor_encoders', self.wheel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for publishing odometry
        self.create_timer(0.01, self.publish_odometry)

    def wheel_callback(self, motor_encoders):
        self.current_time = self.get_clock().now()

        # Assuming the motor_encoders data is [left_front, left_rear, right_front, right_rear]
        for i in range(4):
            if i < 2:  # Left side wheels
                self.delta_left[i] = motor_encoders.data[i] - self._PreviousEncoderCounts[i]
            else:  # Right side wheels
                self.delta_right[i-2] = motor_encoders.data[i] - self._PreviousEncoderCounts[i]

        # Calculate the total ticks for both sides
        self.delta_left_total = sum(self.delta_left)
        self.delta_right_total = sum(self.delta_right)

        self.dt = (self.current_time - self.last_time).to_sec()

        # Update velocities and distances based on the total encoder counts for each side
        for i in range(2):
            self.v_left[i] = self.delta_left_total * self.DistancePerCount / self.dt
            self.v_right[i] = self.delta_right_total * self.DistancePerCount / self.dt

        self.delta_distance = 0.5 * (self.delta_left_total + self.delta_right_total) * self.DistancePerCount
        self.delta_th = (self.delta_right_total - self.delta_left_total) * self.DistancePerCount / (2 * self.wheel_track)

        self.delta_x = self.delta_distance * math.cos(self.th)
        self.delta_y = self.delta_distance * math.sin(self.th)

        # Update odometry based on stepper motors for spot turn and crab crawl
        if self.spot_turn_active:
            # Update only yaw (th) based on encoder data
            self.th += self.delta_th
        elif self.crab_crawl_active:
            # Update only x and y based on encoder data
            self.x += self.delta_x
            self.y += self.delta_y
        else:
            # Consider it as a four-wheeled skid-steering rover and update x, y, th
            self.x += self.delta_x
            self.y += self.delta_y
            self.th += self.delta_th

        # Update encoder counts and time
        self._PreviousEncoderCounts[0] = motor_encoders.data[0]
        self._PreviousEncoderCounts[1] = motor_encoders.data[1]
        self._PreviousEncoderCounts[2] = motor_encoders.data[2]
        self._PreviousEncoderCounts[3] = motor_encoders.data[3]

        self.last_time = self.current_time

    def publish_odometry(self):
        odom_quat = Quaternion()
        odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w = tf2_ros.transformations.quaternion_from_euler(0, 0, self.th)

        # Publish the transform over tf
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.current_time.to_msg()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat

        # Send the transform
        self.odom_broadcaster.sendTransform(odom_trans)

        # Publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = "odom"
        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        odom.pose.covariance = [1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6]

        # Set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.delta_x / self.dt
        odom.twist.twist.linear.y = self.delta_y / self.dt
        odom.twist.twist.angular.z = self.delta_th / self.dt
        odom.twist.covariance = [1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6]

        # Publish the message
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = StepperDiffDrive()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
