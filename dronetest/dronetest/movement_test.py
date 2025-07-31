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
        self.declare_parameter('target_latitude', -7.8332205)
        self.declare_parameter('target_longitude', 110.3844444)
        self.declare_parameter('target2_latitude', -7.8332610)
        self.declare_parameter('target2_longitude', 110.3846580)
        self.declare_parameter('target_altitude', 2.0)
        
        self.connection_string = self.get_parameter('connection_string').get_parameter_value().string_value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').get_parameter_value().double_value
        self.target_lat = self.get_parameter('target_latitude').get_parameter_value().double_value
        self.target_lon = self.get_parameter('target_longitude').get_parameter_value().double_value
        self.target2_lat = self.get_parameter('target2_latitude').get_parameter_value().double_value
        self.target2_lon = self.get_parameter('target2_longitude').get_parameter_value().double_value
        self.target_alt = self.get_parameter('target_altitude').get_parameter_value().double_value


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

    def get_distance_metres(self, aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        This method is an approximation, and will not be accurate over large distances and close to the earth's poles.
        It comes from the ArduPilot test code:
        https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def arm_and_takeoff(self,aTargetAltitude):
        """Arms the vehicle and takes off to the specified altitude."""
        self.get_logger().info("--- Arm and Takeoff Sequence ---")

        # Set mode to GUIDED (mode 4 for Copter) without verification
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

        while True:
            # Wait for the GLOBAL_POSITION_INT message
            msg = self.vehicle.recv_match(type='RANGEFINDER', blocking=True)
            if not msg:
                self.get_logger().error("Did not receive GLOBAL_POSITION_INT message.")
                continue
            
            # altitude is in millimeters
            current_altitude = msg.distance
            self.get_logger().info(f"Altitude: {current_altitude:.2f}m")
            if current_altitude >= aTargetAltitude * 0.90:
                self.get_logger().info("Reached target altitude!")
                break
            time.sleep(1)

    def goto_position(self,lat,lon):
        """Commands the drone to fly to the target GPS coordinate."""
        self.get_logger().info(f"--- Go To Position Sequence ---")
        self.get_logger().info(f"Going to LAT:{lat}, LON:{lon}, ALT:{self.target_alt}m")
        
        # The mode should already be GUIDED from the takeoff sequence.
        # We send the position command directly.
        self.vehicle.mav.set_position_target_global_int_send(
            0, self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            3576, int(self.target_lat * 1e7),
            int(self.target_lon * 1e7), self.target_alt,
            0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(10)
        self.vehicle.mav.set_position_target_global_int_send(
            0, self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            3576, int(self.target2_lat * 1e7),
            int(self.target2_lon * 1e7), self.target_alt,
            0, 0, 0, 0, 0, 0, 0, 0)
        target_location = mavutil.mavlink.MAVLink_global_position_int_message(0,int(lat*1e7),int(lon*1e7),0,0,0,0,0,0)

        #while True:
        #    msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        #    if not msg:
        #        self.get_logger().error("Did not receive GLOBAL_POSITION_INT message.")
        #        continue

        #    current_location = mavutil.mavlink.MAVLink_global_position_int_message(0,msg.lat,msg.lon,0,0,0,0,0,0)
         #   distance = self.get_distance_metres(current_location, target_location)
          #  self.get_logger().info(f"Distance to target: {distance:.2f}m")
           # 
           # if distance <= 1.0: # Set a tolerance of 1 meter
            #    self.get_logger().info("Reached target location!")
             #   break
        time.sleep(10)

    def land(self):
        """Lands the vehicle and waits for it to be on the ground."""
        self.get_logger().info("--- Landing Sequence ---")

        # Set mode to LAND (mode 9 for Copter) without verification
        self.get_logger().info("Setting mode to LAND...")
        land_mode_id = 9
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            land_mode_id)
        time.sleep(1) # Give a moment for the mode change to process

        # Wait for mode change confirmation
        ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.get_logger().info("Mode set to LAND successfully.")
        else:
            self.get_logger().error("Failed to set mode to LAND.")
            return

        # Publish landing status
        land_status = Bool()
        land_status.data = True
        self.landing_pub.publish(land_status)

        # Wait for the drone to land
        self.get_logger().info("Waiting for drone to land...")
        # Check if altitude is very low
        current_altitude = msg.relative_alt / 1000.0
        if current_altitude < 0.2:
            self.get_logger().info("Drone has landed.")
            return True
        time.sleep(1)
        return False

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
    
    drone_controller.get_logger().info("Connection Successfull. Node is Ready")
    input("Press ENTER to start")

    try:
       # --- Mission Execution ---
       drone_controller.arm_and_takeoff(drone_controller.takeoff_altitude)
       # Go to the target coordinates after successful takeoff
       drone_controller.goto_position(drone_controller.target_lat,drone_controller.target_lon)
       # Hold position for a while after reaching target
       drone_controller.get_logger().info("Holding position for 10 seconds.")
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