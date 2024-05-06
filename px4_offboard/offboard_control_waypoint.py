#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus
from nav_msgs.msg import Odometry
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

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile)

        # Timer
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        # Parameters
        # self.declare_parameter('waypoint_x', 50.0)
        # self.declare_parameter('waypoint_y', 50.0)
        # self.declare_parameter('waypoint_z', -15.0)
        self.param_x = 50.0
        self.param_y =  50.0
        self.param_z = 15.0
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
        except Exception as e:
            self.get_logger().error(f"Error processing odometry data: {str(e)}")

    def vehicle_status_callback(self, msg):
        self.get_logger().info(f"NAV_STATUS: {msg.nav_state}, OFFBOARD STATUS: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmdloop_callback(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # waypoint_x = self.get_parameter('waypoint_x').get_parameter_value()
        # waypoint_y = self.get_parameter('waypoint_y').get_parameter_value()
        # waypoint_z = self.get_parameter('waypoint_z').get_parameter_value()
        waypoint_x = self.param_x 
        waypoint_y = self.param_y 
        waypoint_z = self.param_z 


        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            if not self.is_at_waypoint(waypoint_x, waypoint_y, waypoint_z, 1.0):
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.position[0] = waypoint_x
                trajectory_msg.position[1] = waypoint_y
                trajectory_msg.position[2] = waypoint_z
                self.publisher_trajectory.publish(trajectory_msg)
            else:
                self.timer.cancel()  # Stop updating if at the waypoint

    def is_at_waypoint(self, target_x, target_y, target_z, tolerance):
        return (abs(self.current_x - target_x) < tolerance and
                abs(self.current_y - target_y) < tolerance and
                abs(self.current_z - target_z) < tolerance)

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

