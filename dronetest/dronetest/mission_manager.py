#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from dronetest_msgs.msg import ObjectPosition
from std_msgs.msg import Bool

class MissionManager(Node):
    def __init__(self):
        # 1. Initialize the Node
        super().__init__('mission_manager')
        
        # 2. Declare and get parameters
        # For dictionaries, it's often cleaner to declare individual components
        self.declare_parameter('target_gps_lat', -35.363261)
        self.declare_parameter('target_gps_lon', 149.165230)
        self.declare_parameter('landing_altitude', 1.0)
        
        self.target_gps = {
            'lat': self.get_parameter('target_gps_lat').get_parameter_value().double_value,
            'lon': self.get_parameter('target_gps_lon').get_parameter_value().double_value
        }
        self.landing_altitude = self.get_parameter('landing_altitude').get_parameter_value().double_value
        
        # State machine
        self.state = "INIT"
        self.object_centered = False
        self.navigation_timer = 0 # Counter to simulate navigation time
        
        # 3. Create Subscribers
        qos_profile = 10
        self.create_subscription(
            ObjectPosition,
            '/object_position',
            self.object_callback,
            qos_profile)
        
        self.create_subscription(
            Bool,
            '/landing_status',
            self.landing_callback,
            qos_profile)
            
        # 4. Create a timer to run the state machine at 1Hz
        self.timer = self.create_timer(1.0, self.run_state_machine)
        self.get_logger().info("Mission Manager node has been started.")
        
    def object_callback(self, msg):
        self.object_centered = msg.centered if msg.detected else False
        
    def landing_callback(self, msg):
        if msg.data and self.state == "LANDING":
            self.state = "COMPLETE"
            self.get_logger().info("Landing confirmed. Mission complete!")
            
    def run_state_machine(self):
        """This method is called by the timer and executes one step of the state machine."""
        
        if self.state == "INIT":
            self.get_logger().info("State: INIT. Initializing mission...")
            # In a real mission, you might check pre-arm conditions here
            self.state = "TAKEOFF"
            
        elif self.state == "TAKEOFF":
            self.get_logger().info("State: TAKEOFF. Commanding takeoff.")
            # This would trigger a service call to the drone controller
            self.state = "NAVIGATING"
            
        elif self.state == "NAVIGATING":
            # Simulate navigation time without blocking
            if self.navigation_timer == 0:
                self.get_logger().info("State: NAVIGATING. Moving to target GPS coordinates.")
            
            self.navigation_timer += 1
            self.get_logger().info(f"Navigating... ({self.navigation_timer}s)")
            
            if self.navigation_timer >= 5: # Simulate 5 seconds of navigation
                self.get_logger().info("Arrived at target location.")
                self.state = "LANDING"
                
        elif self.state == "LANDING":
            if self.object_centered:
                self.get_logger().info("State: LANDING. Object centered. Commanding precision landing.")
                # This state would now wait for the landing_callback to transition to COMPLETE
            else:
                self.get_logger().info("State: LANDING. Searching for object to land on...")
                
        elif self.state == "COMPLETE":
            self.get_logger().info("State: COMPLETE. Shutting down mission manager.")
            self.timer.cancel()  # Stop the timer
            rclpy.shutdown()     # Shut down the node
            
# 5. Main execution block for ROS 2
def main(args=None):
    rclpy.init(args=args)
    mission_manager = MissionManager()
    try:
        # spin() keeps the node alive so the timer and subscribers can work.
        rclpy.spin(mission_manager)
    except KeyboardInterrupt:
        pass
    finally:
        mission_manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()