#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from drone_interfaces.msg import DroneState, MissionStatus
from mavros_msgs.msg import State, ExtendedState, StatusText, Altitude
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Header, Float64
import math

class PX4IntegrationNode(Node):
    def __init__(self):
        super().__init__('px4_integration_node')
        
        # Define QoS profile to match MAVROS
        # MAVROS typically uses BEST_EFFORT reliability for sensor data
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # For state messages, MAVROS might use RELIABLE
        self.state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers for our custom messages
        self.drone_state_pub = self.create_publisher(
            DroneState, '/drone/state', 10)
        
        # Publisher for testing control
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Subscribers to MAVROS topics with appropriate QoS
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.mavros_state_callback, 
            qos_profile=self.state_qos)
        
        self.extended_state_sub = self.create_subscription(
            ExtendedState, '/mavros/extended_state', 
            self.extended_state_callback, 
            qos_profile=self.state_qos)
        
        # Use sensor QoS for sensor data topics
        self.altitude_sub = self.create_subscription(
            Altitude, '/mavros/altitude',
            self.altitude_callback, 
            qos_profile=self.sensor_qos)
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', 
            self.pose_callback, 
            qos_profile=self.sensor_qos)
        
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self.velocity_callback, 
            qos_profile=self.sensor_qos)
        
        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery',
            self.battery_callback, 
            qos_profile=self.sensor_qos)
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.gps_callback, 
            qos_profile=self.sensor_qos)
        
        self.compass_sub = self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg',
            self.compass_callback, 
            qos_profile=self.sensor_qos)
        
        self.statustext_sub = self.create_subscription(
            StatusText, '/mavros/statustext/recv',
            self.statustext_callback, 
            qos_profile=self.sensor_qos)
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # State storage
        self.current_drone_state = DroneState()
        self.mavros_state = None
        self.extended_state = None
        self.altitude_data = None
        self.has_battery_data = False
        self.has_pose_data = False
        self.has_gps_data = False
        
        # Timer to publish drone state
        self.timer = self.create_timer(0.5, self.publish_drone_state)
        
        # Timer for setpoint publishing (needed for OFFBOARD mode)
        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint)  # 20Hz
        self.current_setpoint = PoseStamped()
        
        # Status timer
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('='*50)
        self.get_logger().info('PX4 Integration Node Started!')
        self.get_logger().info('Waiting for MAVROS connection...')
        self.get_logger().info('='*50)
        
    def print_status(self):
        """Print current status"""
        if self.mavros_state and self.mavros_state.connected:
            status = f"\n--- Drone Status ---"
            status += f"\n  Connected: {self.mavros_state.connected}"
            status += f"\n  Mode: {self.mavros_state.mode}"
            status += f"\n  Armed: {self.mavros_state.armed}"
            
            if self.has_pose_data:
                pos = self.current_drone_state.current_pose.position
                status += f"\n  Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}"
                status += f"\n  Heading: {self.current_drone_state.heading_deg:.1f}°"
            
            if self.altitude_data:
                status += f"\n  Altitude (rel): {self.altitude_data.relative:.2f}m"
                
            if self.has_battery_data:
                status += f"\n  Battery: {self.current_drone_state.battery_percentage:.1f}%"
            else:
                status += f"\n  Battery: No data (normal in SITL)"
                
            self.get_logger().info(status)
        
    def mavros_state_callback(self, msg):
        old_state = self.mavros_state
        self.mavros_state = msg
        
        # Log connection changes
        if old_state is None or old_state.connected != msg.connected:
            if msg.connected:
                self.get_logger().info(f'✓ Connected to PX4!')
                self.get_logger().info(f'  System ID: {msg.system_status}')
                self.get_logger().info(f'  Mode: {msg.mode}')
                self.get_logger().info(f'  Armed: {msg.armed}')
            else:
                self.get_logger().warn('✗ Disconnected from PX4!')
                
        # Log mode changes
        if old_state and old_state.mode != msg.mode:
            self.get_logger().info(f'Mode changed: {old_state.mode} → {msg.mode}')
            
        # Log arming changes
        if old_state and old_state.armed != msg.armed:
            if msg.armed:
                self.get_logger().info('🚁 Drone ARMED')
            else:
                self.get_logger().info('🛑 Drone DISARMED')
        
    def extended_state_callback(self, msg):
        self.extended_state = msg
        
    def altitude_callback(self, msg):
        self.altitude_data = msg
        self.current_drone_state.altitude_relative = msg.relative
        self.current_drone_state.altitude_msl = msg.amsl
        
    def pose_callback(self, msg):
        self.has_pose_data = True
        self.current_drone_state.current_pose = msg.pose
        
        # Also use this for altitude if altitude topic isn't available
        if not self.altitude_data:
            self.current_drone_state.altitude_relative = msg.pose.position.z
        
        # Calculate heading from quaternion
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_drone_state.heading_deg = math.degrees(yaw)
        
    def velocity_callback(self, msg):
        self.current_drone_state.current_velocity = msg.twist
        
    def battery_callback(self, msg):
        self.has_battery_data = True
        self.current_drone_state.battery_voltage = msg.voltage
        self.current_drone_state.battery_percentage = msg.percentage * 100.0
        
    def gps_callback(self, msg):
        self.has_gps_data = True
        self.current_drone_state.gps_fix = msg
        
    def compass_callback(self, msg):
        # Alternative heading source
        pass  # We're already calculating from quaternion
        
    def statustext_callback(self, msg):
        # Log important status messages from PX4
        severity_map = {
            0: "EMERGENCY",
            1: "ALERT",
            2: "CRITICAL",
            3: "ERROR",
            4: "WARNING",
            5: "NOTICE",
            6: "INFO",
            7: "DEBUG"
        }
        severity = severity_map.get(msg.severity, "UNKNOWN")
        
        if msg.severity <= 5:  # Show up to NOTICE level
            self.get_logger().info(f'[PX4-{severity}] {msg.text}')
        
    def publish_drone_state(self):
        if self.mavros_state is None:
            return
            
        # Update header
        self.current_drone_state.header = Header()
        self.current_drone_state.header.stamp = self.get_clock().now().to_msg()
        self.current_drone_state.header.frame_id = "map"
        
        # Update fields from MAVROS state
        self.current_drone_state.drone_id = "px4_sitl_drone"
        self.current_drone_state.mode = self.mavros_state.mode
        self.current_drone_state.armed = self.mavros_state.armed
        self.current_drone_state.connected = self.mavros_state.connected
        
        # If no battery data (common in SITL), set defaults
        if not self.has_battery_data:
            self.current_drone_state.battery_percentage = -1.0
            self.current_drone_state.battery_voltage = 12.6
        
        # Publish
        self.drone_state_pub.publish(self.current_drone_state)
        
    def publish_setpoint(self):
        """Publish setpoint for OFFBOARD mode"""
        # Update timestamp
        self.current_setpoint.header.stamp = self.get_clock().now().to_msg()
        self.current_setpoint.header.frame_id = "map"
        
        # Set a hover position if not set
        if self.current_setpoint.pose.position.z == 0.0:
            self.current_setpoint.pose.position.x = 0.0
            self.current_setpoint.pose.position.y = 0.0
            self.current_setpoint.pose.position.z = 2.0  # 2 meters altitude
            self.current_setpoint.pose.orientation.w = 1.0
            
        self.setpoint_pub.publish(self.current_setpoint)

def main(args=None):
    rclpy.init(args=args)
    node = PX4IntegrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\nShutdown requested...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
