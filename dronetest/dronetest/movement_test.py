#!/usr/bin/env python
import rclpy
import time
import math
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import Bool

class DroneController(Node):
    """
    A ROS2 node to control a drone using MAVLink for a simple mission:
    takeoff, go to a waypoint, and land.
    """
    def __init__(self):
        # 1. Initialize the Node
        super().__init__('drone_controller')
        
        # 2. Declare and get parameters for connection and mission
        self.declare_parameter('connection_string', 'udp:127.0.0.1:14551')
        self.declare_parameter('takeoff_altitude', 2.0)

        self.declare_parameter('waypoint1_lat', -7.8332205)
        self.declare_parameter('waypoint1_lon', 110.3844444)
        self.declare_parameter('waypoint2_lat', -7.8332610)
        self.declare_parameter('waypoint2_lon', 110.3846580)
    
        self.connection_string = self.get_parameter('connection_string').value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.wp1_lat = self.get_parameter('waypoint1_lat').value
        self.wp1_lon = self.get_parameter('waypoint1_lon').value
        self.wp2_lat = self.get_parameter('waypoint2_lat').value
        self.wp2_lon = self.get_parameter('waypoint2_lon').value

        self.vehicle = None # Initialize vehicle to None

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
        self.landing_pub = self.create_publisher(Bool, '/landing_status', 10)

    def get_distance_haversine_int(self, location_int_1, location_int_2):
        """
        Calculates distance between two MAVLink_global_position_int_message objects.
        """
        R = 6371000  # Radius of Earth in meters
        lat1 = location_int_1.lat / 1e7
        lon1 = location_int_1.lon / 1e7
        lat2 = location_int_2.lat / 1e7
        lon2 = location_int_2.lon / 1e7

        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
        dlon_rad, dlat_rad = lon2_rad - lon1_rad, lat2_rad - lat1_rad

        a = math.sin(dlat_rad / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon_rad / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def arm_and_takeoff(self,target_altitude):
        """Arms the vehicle and takes off to the specified altitude."""
        self.get_logger().info("--- Arm and Takeoff Sequence ---")

        # Set mode to GUIDED (mode 4 for Copter)
        self.get_logger().info("Setting mode to GUIDED...")
        guided_mode_id = 4
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_mode_id)
        time.sleep(1) # Give a moment for the mode change to process

        # Arm the vehicle
        self.get_logger().info("Sending arm command...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0) # 1 to arm

        self.get_logger().info("Waiting for vehicle to arm...")
        self.vehicle.motors_armed_wait()
        self.get_logger().info("Vehicle armed!")

        # Send takeoff command
        self.get_logger().info(f"Sending takeoff command to {self.takeoff_altitude}m")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, self.takeoff_altitude)
        time.sleep(1) # Give drone a moment to start ascending

        while True:
            # Wait for the GLOBAL_POSITION_INT message
            msg = self.vehicle.recv_match(type='RANGEFINDER', blocking=True)
            if not msg:
                self.get_logger().error("Did not receive GLOBAL_POSITION_INT message.")
                continue
            
            # altitude is in millimeters
            current_altitude = msg.distance
            self.get_logger().info(f"Altitude: {current_altitude:.2f}m")
            if current_altitude >= target_altitude * 0.90:
                self.get_logger().info("Reached target altitude!")
                break

    def goto_position(self, lat, lon, alt, timeout=30):
        """
        Commands the drone to a specific lat/lon/alt and waits for arrival.
        Returns True on success, False on timeout.
        """
        self.get_logger().info(f"--- Commanding drone to LAT:{lat}, LON:{lon}, ALT:{alt}m ---")
        
        # Send the command to go to the waypoint
        self.vehicle.mav.set_position_target_global_int_send(
            0, self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            3576, # type_mask 3576 (use position only)
            int(lat * 1e7),
            int(lon * 1e7),
            alt, 0, 0, 0, 0, 0, 0, 0, 0)
        
        target_location = mavutil.mavlink.MAVLink_global_position_int_message(0,int(lat*1e7),int(lon*1e7),0,0,0,0,0,0)
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if not msg:
                self.get_logger().warn("No position update received in 1 second.")
                continue

            distance = self.get_distance_haversine_int(msg, target_location)
            self.get_logger().info(f"Distance to target: {distance:.2f}m")
            
            if distance <= 1.5: # Arrival tolerance of 1.5 meters
                self.get_logger().info("Reached target location!")
                return True

        self.get_logger().error(f"Failed to reach target within {timeout} seconds.")
        return False

    def land(self):
        """Lands the vehicle and confirms it has landed."""
        self.get_logger().info("--- Landing Sequence ---")
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9) # 9 is LAND for Copter
        
        # Wait until the vehicle is on the ground
        while True:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if not msg: continue
            
            # Check if altitude is near zero and not moving
            if msg.relative_alt < 200: # relative_alt is in mm, so 200mm = 0.2m
                self.get_logger().info("Drone has landed.")
                land_status = Bool()
                land_status.data = True
                self.landing_pub.publish(land_status)
                break
            time.sleep(1)

    def disarm(self):
        """Disarms the vehicle."""
        self.get_logger().info("--- Disarming ---")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0) # 0 to disarm

        self.get_logger().info("Waiting for vehicle to disarm...")
        self.vehicle.motors_disarmed_wait()
        self.get_logger().info("Vehicle disarmed!")
        return True

    def close_connection(self):
        """Safely close the vehicle connection."""
        if self.vehicle:
            self.get_logger().info("Closing vehicle connection.")
            self.vehicle.close()

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    
    # Exit if the connection failed in the constructor
    if not rclpy.ok():
        return
    
    drone_controller.get_logger().info("Connection Successfull. Mission is Ready")
    input("Press ENTER to start")

    try:
        # <<< MODIFIED >>> Use the new function for multiple waypoints
        drone_controller.get_logger().info("--- Flying to Waypoint 1 ---")
        if drone_controller.goto_position(drone_controller.wp1_lat, drone_controller.wp1_lon, drone_controller.takeoff_altitude):
            drone_controller.get_logger().info("Holding position at Waypoint 1 for 5 seconds.")
            time.sleep(5)
        
        drone_controller.get_logger().info("--- Flying to Waypoint 2 ---")
        if drone_controller.goto_position(drone_controller.wp2_lat, drone_controller.wp2_lon, drone_controller.takeoff_altitude):
            drone_controller.get_logger().info("Holding position at Waypoint 2 for 5 seconds.")
            time.sleep(5)

    except KeyboardInterrupt:
        drone_controller.get_logger().info('Keyboard interrupt, initiating landing.')
    except Exception as e:
        drone_controller.get_logger().error(f"An unhandled exception occurred during mission: {e}")
    finally:
        # 4. Land, disarm, and clean up
        drone_controller.get_logger().info("--- Mission End or Interrupt: Cleaning up ---")
        drone_controller.land()
        drone_controller.disarm()
        drone_controller.close_connection()
        drone_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
 