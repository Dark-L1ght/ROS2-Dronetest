#!/usr/bin/env python
import rclpy
import time
import math
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import Bool

class DroneController(Node):
    def __init__(self):
        # 1. Initialize the Node
        super().__init__('drone_controller')
        
        # 2. Declare and get parameters for connection and mission
        self.declare_parameter('connection_string', '/dev/ttyS4,921600')
        self.declare_parameter('takeoff_altitude', 2.0)
        self.declare_parameter('target_latitude', -7.8332903)
        self.declare_parameter('target_longitude', 110.3843720)
        self.declare_parameter('target_altitude', 2.0)
        
        self.connection_string = self.get_parameter('connection_string').get_parameter_value().string_value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').get_parameter_value().double_value
        self.target_lat = self.get_parameter('target_latitude').get_parameter_value().double_value
        self.target_lon = self.get_parameter('target_longitude').get_parameter_value().double_value
        self.target_alt = self.get_parameter('target_altitude').get_parameter_value().double_value

        # 3. Pymavlink Connection
        self.get_logger().info(f"Connecting to vehicle on: {self.connection_string}")
        try:
            self.vehicle = mavutil.mavlink_connection(self.connection_string, wait_ready=True)
            self.vehicle.wait_heartbeat()
            self.get_logger().info("Heartbeat from system (system %u component %u)" % (self.vehicle.target_system, self.vehicle.target_component))
        except Exception as e:
            self.get_logger().error(f"Failed to connect to vehicle: {e}")
            rclpy.shutdown()
            return

        # 4. Create Publishers
        # This publisher can be used to signal other nodes that the drone is landing
        self.landing_pub = self.create_publisher(Bool, '/landing_status', 10)

    def set_mode(self, mode):
        """Set the vehicle mode using MAVLink."""
        if mode not in self.vehicle.mode_mapping():
            self.get_logger().error(f"Unknown mode: {mode}")
            return False

        # Wait for mode change confirmation
        ack_msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.get_logger().info(f"Mode change to {mode} accepted.")
                return True
            else:
                self.get_logger().error(f"Mode change to {mode} failed.")
                return False
        return False


    def arm_and_takeoff(self):
        """Arms the vehicle and takes off to the specified altitude."""
        self.get_logger().info("Arming and taking off")

        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4)

        # if not self.set_mode("GUIDED"):
        #     return

        # Arm the vehicle
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0) # 1 to arm

        self.get_logger().info("Waiting for vehicle to arm...")
        self.vehicle.motors_armed_wait()
        self.get_logger().info("Vehicle armed!")

        # Send takeoff command
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, self.takeoff_altitude)

        #Wait until the vehicle reaches the target altitude
       while rclpy.ok():
           rangefinder_data = self.vehicle.recv_match(type='RANGEFINDER', blocking=True)
           self.get_logger().info(f"Current altitude: {rangefinder_data}m")
           if rangefinder_data >= self.takeoff_altitude * 0.95:
               self.get_logger().info("Reached target altitude")
               break
           time.sleep(0.5)

    def goto_position(self):
        """Commands the drone to fly to the target GPS coordinate."""
        self.get_logger().info(f"Going to LAT:{self.target_lat}, LON:{self.target_lon}, ALT:{self.target_alt}m")
        
        # Set the drone to GUIDED mode
        # if not self.set_mode("GUIDED"):
        #     return

        # Send the command to go to the target location
        self.vehicle.mav.set_position_target_global_int_send(
            0,       # time_boot_ms (not used)
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111000111, # type_mask (use position only)
            int(self.target_lat * 1e7), # latitude in degrees * 1e7
            int(self.target_lon * 1e7), # longitude in degrees * 1e7
            self.target_alt,            # altitude in meters
            0, 0, 0, # velocity components (not used)
            0, 0, 0, # acceleration components (not used)
            0, 0)    # yaw, yaw_rate (not used)

        # Wait until the drone reaches the target location
        while rclpy.ok():
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            
            # Simple distance calculation (Pythagorean theorem on a flat Earth projection)
            # This is a rough approximation and works for short distances.
            d_lat = (lat - self.target_lat) * 1.113195e5
            d_lon = (lon - self.target_lon) * 1.113195e5 * math.cos(math.radians(self.target_lat))
            distance = math.sqrt(d_lat**2 + d_lon**2)
            
            self.get_logger().info(f"Distance to target: {distance:.2f}m")
            if distance < 2.0: # Consider it arrived if within 2 meters
                self.get_logger().info("Arrived at target coordinates.")
                break
            time.sleep(1)

    def land(self):
        """Lands the vehicle."""
        self.get_logger().info("Landing")
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            9)
        land_status = Bool()
        land_status.data = True
        self.landing_pub.publish(land_status)

    def close_connection(self):
        """Safely close the vehicle connection."""
        if self.vehicle:
            self.get_logger().info("Closing vehicle connection.")
            self.vehicle.close()

    def disarm(self):
        """Arms the vehicle and takes off to the specified altitude."""
        self.get_logger().info("Disarming")
        
        # if not self.set_mode("GUIDED"):
        #     return

        # Disarm the vehicle
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0) # 0 to arm

        self.get_logger().info("Waiting for vehicle to disarm...")
        self.vehicle.motors_armed_wait()
        self.get_logger().info("Vehicle disarmed!")

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    
    # Exit if the connection failed in the constructor
    if not rclpy.ok():
        return

    try:
        # --- Mission Execution ---
        # 1. Arm and take off
        drone_controller.arm_and_takeoff()
        time.sleep(15) # Short pause after takeoff
        drone_controller.land()
        time.sleep(5) # Give time for the land command to be processed
        # # 2. Go to the target coordinates
        # drone_controller.goto_position()

        # # 3. Hold position for a while
        # drone_controller.get_logger().info("Holding position for 15 seconds.")
        # time.sleep(15)

    except KeyboardInterrupt:
        drone_controller.get_logger().info('Keyboard interrupt, initiating landing.')
    except Exception as e:
        drone_controller.get_logger().error(f"An unhandled exception occurred during mission: {e}")
    finally:
        # 4. Land and clean up

        drone_controller.close_connection()
        drone_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
