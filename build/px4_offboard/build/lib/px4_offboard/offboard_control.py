#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus

from geometry_msgs.msg import Quaternion

class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        timer_period = 0.02 # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback) #calls the cmdloop for the specified timer_period
        self.dt = timer_period

        self.declare_parameter('target_x', 150.0)
        self.declare_parameter('target_y', 105.0)
        self.declare_parameter('target_z', -115.0)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_orientation = Quaternion()

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.get_logger().info(f"NAV_STATUS: , {msg.nav_state}- offboard status: , {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.get_logger().info(f"Position: x={self.current_x}, y={self.current_y}, altitude={self.current_z}")

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            # yaw_angle = self.quaternion_to_yaw(self.current_orientation)
            

            target_x= self.get_parameter('target_x').value
            target_y = self.get_parameter('target_y').value
            target_z = self.get_parameter('target_z').value

            direction_x = target_x - self.current_x
            direction_y = target_y - self.current_y
            direction_z = target_z - self.current_z

            # Normalize the direction vector (to get a unit vector)
            norm = np.sqrt(direction_x**2 + direction_y**2 + direction_z**2)
            if norm > 0:
                direction_x /= norm
                direction_y /= norm
                direction_z /= norm

            # Set the trajectory message to move towards the target
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.current_x + direction_x * self.dt * 10  # Scale the step size
            trajectory_msg.position[1] = self.current_y + direction_y * self.dt * 10
            trajectory_msg.position[2] = self.current_z + direction_z * self.dt * 10

            self.publisher_trajectory.publish(trajectory_msg)

            # Update current position for next iteration (simple simulation)
            self.current_x += direction_x * self.dt * 10
            self.current_y += direction_y * self.dt * 10
            self.current_z += direction_z * self.dt * 10




def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()