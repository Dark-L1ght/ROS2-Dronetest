#!/usr/bin/env python
import rclpy
import time
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative
from geometry_msgs.msg import Twist
from dronetest_msgs.msg import ObjectPosition
from std_msgs.msg import Bool

class DroneController(Node):
    def __init__(self):
        # 1. Initialize the Node
        super().__init__('drone_controller')
        
        # 2. Declare and get parameters
        self.declare_parameter('connection_string', '/dev/ttyS4,921600')
        self.declare_parameter('default_altitude', 5.0)
        self.declare_parameter('simulation_mode', False) # Added for clarity
        
        self.connection_string = self.get_parameter('connection_string').get_parameter_value().string_value
        self.default_altitude = self.get_parameter('default_altitude').get_parameter_value().double_value
        self.simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value

        self.get_logger().info(f"Connecting to vehicle on: {self.connection_string}")
        try:
            # Drone connection
            self.vehicle = connect(self.connection_string, wait_ready=True, timeout=60)
            self.get_logger().info("Vehicle connected successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to vehicle: {e}")
            # Exit if connection fails
            rclpy.shutdown()
            return

        # 3. Create Publishers and Subscribers
        qos_profile = 10
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.landing_pub = self.create_publisher(Bool, '/landing_status', qos_profile)
        self.object_sub = self.create_subscription(
            ObjectPosition,
            '/object_position',
            self.object_callback,
            qos_profile)
        
        # Control parameters
        self.k_p = 0.01  # Proportional gain for position correction
        self.landing_speed = 0.3  # m/s
        
    def arm_and_takeoff(self, altitude):
        self.get_logger().info("Arming and taking off")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        
        while not self.vehicle.armed:
            time.sleep(0.1) # Use time.sleep
        
        self.vehicle.simple_takeoff(altitude)
        
        while self.vehicle.location.global_relative_frame.alt < altitude * 0.95:
            self.get_logger().info(f"Current altitude: {self.vehicle.location.global_relative_frame.alt:.2f}m")
            time.sleep(0.5)
        
        self.get_logger().info("Reached target altitude")
        
    def goto_position(self, lat, lon, alt):
        self.get_logger().info(f"Going to position: {lat}, {lon} at {alt}m")
        target = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(target)
        
    def object_callback(self, msg):
        if not msg.detected or self.vehicle.mode.name != "GUIDED":
            return
            
        cmd = Twist()
        
        if msg.centered:
            # Descend
            cmd.linear.z = -self.landing_speed
            self.get_logger().info("Object centered - descending")
        else:
            # Adjust position (NED frame -> Drone Frame)
            # dy (North) -> +vx (forward)
            # dx (East)  -> -vy (left)
            cmd.linear.x = -msg.dy * self.k_p
            cmd.linear.y = -msg.dx * self.k_p
            self.get_logger().info(f"Adjusting position: dx={msg.dx:.1f}, dy={msg.dy:.1f}")
        
        # In ROS 2, it's safer to use the explicit method
        self.send_nav_command(cmd.linear.x, cmd.linear.y, cmd.linear.z, 0.0)
        
    def land(self):
        self.get_logger().info("Landing")
        self.vehicle.mode = VehicleMode("LAND")
        land_status = Bool()
        land_status.data = True
        self.landing_pub.publish(land_status)

    def send_nav_command(self, vx, vy, vz, rot):
        """
        Sends velocity commands to the vehicle.
        This example assumes Twist messages are for simulation and a MAVLink message would be used for a real drone.
        """
        if self.simulation_mode:
            # For simulation using Twist messages
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz
            twist.angular.z = rot
            self.cmd_vel_pub.publish(twist)
        else:
            # For a real drone, you would send a MAVLink SET_POSITION_TARGET_LOCAL_NED message
            # This is a placeholder for the actual implementation
            self.get_logger().warn("send_nav_command not implemented for physical drone yet.")
            pass
            
    def close_connection(self):
        """Safely close the vehicle connection."""
        if self.vehicle:
            self.get_logger().info("Closing vehicle connection.")
            self.vehicle.close()

# 4. Main execution block for ROS 2
def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    
    # If connection failed, the node will shutdown.
    if not rclpy.ok():
        return

    try:
        # Spin keeps the node alive to process callbacks.
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure resources are cleaned up.
        drone_controller.land()
        drone_controller.close_connection()
        drone_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()