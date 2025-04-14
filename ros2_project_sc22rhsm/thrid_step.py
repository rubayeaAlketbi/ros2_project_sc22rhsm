import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Message type for publishing alerts
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class ColourIdentifier(Node):
    def __init__(self):
        super().__init__('colour_identifier')

        # Sensitivity for green detection
        self.sensitivity = 10
        self.green_detected = False

        # CvBridge setup
        self.bridge = CvBridge()

        # Publisher for messages
        self.publisher = self.create_publisher(String, 'detected_color', 10)

        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.callback, 10)

    def callback(self, data):
        try:
            # Convert ROS Image to OpenCV format
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Convert to HSV
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Define green range
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

            # Create mask
            mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

            # Find contours
            contours, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                contour_area = cv2.contourArea(largest_contour)

                # Ignore small noise
                if contour_area > 500:
                    # Draw bounding box
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Publish or print message
                    if not self.green_detected:
                        self.green_detected = True
                        msg = String()
                        msg.data = "Green detected!"
                        self.publisher.publish(msg)
                        print("Green detected!")

            else:
                self.green_detected = False  # Reset flag

            # Show image
            cv2.imshow('Camera Feed', image)
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")


def main():
    rclpy.init()
    node = ColourIdentifier()
    rclpy.spin(node)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
