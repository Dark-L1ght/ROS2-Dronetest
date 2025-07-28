#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from dronetest_msgs.msg import BlobDetection, ObjectPosition

class ObjectDetector(Node):
    def __init__(self):
        # 1. Initialize the Node
        super().__init__('object_detector')
        
        # 2. Declare and get parameters
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('target_tolerance', 20)
        
        frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.target_tolerance = self.get_parameter('target_tolerance').get_parameter_value().integer_value
        
        # Target position (center of frame)
        self.target_x = frame_width / 2.0
        self.target_y = frame_height / 2.0
        
        self.get_logger().info(f"Targeting center: ({self.target_x}, {self.target_y})px with tolerance: {self.target_tolerance}px")
        
        # 3. Create Publishers and Subscribers
        qos_profile = 10 # Quality of Service profile
        self.object_pub = self.create_publisher(ObjectPosition, '/object_position', qos_profile)
        self.blob_sub = self.create_subscription(
            BlobDetection, 
            '/blob_detection', 
            self.blob_callback, 
            qos_profile)
        
    def blob_callback(self, msg):
        obj_msg = ObjectPosition()
        
        if msg.detected:
            # Calculate position differences
            dx = msg.x - self.target_x
            dy = msg.y - self.target_y
            
            obj_msg.detected = True
            obj_msg.x = msg.x
            obj_msg.y = msg.y
            obj_msg.dx = dx
            obj_msg.dy = dy
            obj_msg.centered = (abs(dx) < self.target_tolerance and 
                              abs(dy) < self.target_tolerance)
        else:
            # If no blob is detected, ensure the message reflects this
            obj_msg.detected = False
            obj_msg.x = 0.0
            obj_msg.y = 0.0
            obj_msg.dx = 0.0
            obj_msg.dy = 0.0
            obj_msg.centered = False
            
        self.object_pub.publish(obj_msg)

# 4. Main execution block for ROS 2
def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    try:
        rclpy.spin(object_detector)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        object_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()