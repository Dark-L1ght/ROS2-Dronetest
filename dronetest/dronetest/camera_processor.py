#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dronetest_msgs.msg import BlobDetection

class CameraProcessor(Node):
    def __init__(self):
        # 1. Initialize the Node
        super().__init__('camera_processor')
        
        # 2. Declare and get parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('hsv_min', [77, 40, 0])
        self.declare_parameter('hsv_max', [101, 255, 255])
        
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.hsv_min = self.get_parameter('hsv_min').get_parameter_value().integer_array_value
        self.hsv_max = self.get_parameter('hsv_max').get_parameter_value().integer_array_value
        
        # Log the parameters being used
        self.get_logger().info(f"Subscribing to camera topic: '{self.camera_topic}'")
        self.get_logger().info(f"Using HSV min values: {self.hsv_min}")
        self.get_logger().info(f"Using HSV max values: {self.hsv_max}")
        
        self.bridge = CvBridge()
        
        # 3. Create Publishers and Subscribers
        qos_profile = 10 # Quality of Service profile
        self.image_pub = self.create_publisher(Image, '/processed_image', qos_profile)
        self.blob_pub = self.create_publisher(BlobDetection, '/blob_detection', qos_profile)
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile)
            
    def apply_search_window(self, image, window=[0.0, 0.0, 1.0, 1.0]):
        rows, cols = image.shape[:2]
        x_min, y_min = int(cols * window[0]), int(rows * window[1])
        x_max, y_max = int(cols * window[2]), int(rows * window[3])
        # Create a black mask of the same size as the image
        mask = np.zeros_like(image)
        # Copy the region of interest from the original image to the mask
        mask[y_min:y_max, x_min:x_max] = image[y_min:y_max, x_min:x_max]
        return mask
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Blob detection (logic is unchanged)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array(self.hsv_min), np.array(self.hsv_max))
            mask = cv2.dilate(mask, None, iterations=2)
            mask = cv2.erode(mask, None, iterations=2)
            mask = self.apply_search_window(mask)
            
            # Find contours
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            blob_msg = BlobDetection()
            blob_msg.detected = False # Default to false

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)

                # Only publish if the blob is of a reasonable size
                if radius > 5:
                    blob_msg.detected = True
                    blob_msg.x = float(x)
                    blob_msg.y = float(y)
                    blob_msg.radius = float(radius)
                    
                    # Draw detection on image
                    cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(cv_image, (int(x), int(y)), 5, (0, 0, 255), -1)
            
            # Publish results
            self.blob_pub.publish(blob_msg)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            
        except Exception as e:
            # 4. Use the node's logger
            self.get_logger().error(f"Image processing error: {e}")

# 5. Main execution block for ROS 2
def main(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()
    try:
        rclpy.spin(camera_processor)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        camera_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()