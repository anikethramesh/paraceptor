#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleCommand

class ReconControl(Node):
    def __init__(self, namespace):
        super().__init__('minimal_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'/{namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, f'/{namespace}/fmu/in/vehicle_command', 10)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, f'/{namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, f'/{namespace}/fmu/in/trajectory_setpoint', qos_profile)

        timer_period = 0.02 # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback) #calls the cmdloop for the specified timer_period
        self.dt = timer_period
        self.declare_parameter('radius', 10.0)
        self.declare_parameter('omega', 5.0)
        self.declare_parameter('altitude', 5.0)
        # NED coords
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value


        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
   

        self.arming_timer = self.create_timer(10.0, self.arm_vehicle) # will activate function after 10 secs
        
    def arm_vehicle(self):
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            arm_command = VehicleCommand()
            arm_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            arm_command.param1 = 1.0 # means arm
            arm_command.confirmation = 0 # no futher confirmation
            arm_command.from_external = True
            self.vehicle_command_publisher_.publish(arm_command)
            self.get_logger().info('RECON Vehicle armed.')

    def vehicle_status_callback(self, msg):
        self.get_logger().info(f"RECON NAV_STATUS: , {msg.nav_state}- offboard status: , {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        # self.get_logger().info(f"Position: x={self.current_x}, y={self.current_y}, altitude={self.current_z}")

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):


            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -self.altitude
            self.publisher_trajectory.publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.dt


def main(args=None):
    rclpy.init(args=args)
    namespace = 'px4_1'
    recon_control = ReconControl(namespace=namespace)

    rclpy.spin(recon_control)

    recon_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()