#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from geometry_msgs.msg import Twist

class OffboardControl(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile)

        # Publisher
        self.publisher_twist = self.create_publisher(Twist, '/fmu/in/setpoint_velocity', qos_profile)

        # Timer
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        # Parameters
        self.declare_parameter('waypoint_x', 100.0)
        self.declare_parameter('waypoint_y', 50.0)
        self.declare_parameter('waypoint_z', -150.0)
        self.declare_parameter('yaw_angle', 0.785398)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_orientation = Quaternion()

    def odometry_callback(self, msg):
        try:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            self.current_z = msg.pose.pose.position.z
            self.current_orientation = msg.pose.pose.orientation
            yaw_angle = self.quaternion_to_yaw(self.current_orientation)

            # Log the current position and direction
            self.get_logger().info(f"Position: x={self.current_x}, y={self.current_y}, altitude={self.current_z}, yaw={yaw_angle} radians")

        except Exception as e:
            self.get_logger().error(f"Error processing odometry data: {str(e)}")

    def vehicle_status_callback(self, msg):
        self.get_logger().info(f"NAV_STATUS: {msg.nav_state}, OFFBOARD STATUS: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmdloop_callback(self):
        # Fetch waypoint parameters
        waypoint_x = self.get_parameter('waypoint_x').get_parameter_value().double_value
        waypoint_y = self.get_parameter('waypoint_y').get_parameter_value().double_value
        waypoint_z = self.get_parameter('waypoint_z').get_parameter_value().double_value
        yaw_angle = self.get_parameter('yaw_angle').get_parameter_value().double_value

        # Calculate velocity vector towards waypoint
        diff_x = waypoint_x - self.current_x
        diff_y = waypoint_y - self.current_y
        diff_z = waypoint_z - self.current_z

        # Normalize velocity for movement
        magnitude = (diff_x**2 + diff_y**2 + diff_z**2)**0.5
        if magnitude > 0:
            linear_x = diff_x / magnitude
            linear_y = diff_y / magnitude
            linear_z = diff_z / magnitude
        else:
            linear_x = linear_y = linear_z = 0.0

        # Set the yaw angular velocity
        yaw_velocity = 0.2  # Set a fixed angular speed for demonstration purposes

        # Construct Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.linear.z = linear_z
        twist_msg.angular.z = yaw_velocity

        # Publish Twist message
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.publisher_twist.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
