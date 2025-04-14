import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')

        # Sensitivity for color detection
        self.sensitivity = 10
        self.green_detected = False
        self.red_detected = False

        # Movement commands
        self.move_forward = Twist()
        self.move_forward.linear.x = 0.2  # Move forward

        self.stop_movement = Twist()  # Stop

        # CvBridge setup
        self.bridge = CvBridge()

        # Publisher for movement
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.callback, 10)

    def callback(self, data):
        try:
            # Convert ROS Image to OpenCV format
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Convert to HSV
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Define color ranges
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

            hsv_red_lower1 = np.array([0, 100, 100])
            hsv_red_upper1 = np.array([10, 255, 255])
            hsv_red_lower2 = np.array([170, 100, 100])
            hsv_red_upper2 = np.array([180, 255, 255])

            # Create masks
            mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            mask_red1 = cv2.inRange(hsv_image, hsv_red_lower1, hsv_red_upper1)
            mask_red2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            # Find contours
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Green detection
            if contours_green:
                largest_contour = max(contours_green, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 500:
                    self.green_detected = True
                    cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 2)
                else:
                    self.green_detected = False

            # Red detection
            if contours_red:
                largest_contour = max(contours_red, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 500:
                    self.red_detected = True
                    cv2.drawContours(image, [largest_contour], -1, (0, 0, 255), 2)
                else:
                    self.red_detected = False

            # Decide movement
            if self.green_detected and not self.red_detected:
                self.publisher.publish(self.move_forward)
                print("Moving Forward - Following Green")
            elif self.red_detected:
                self.publisher.publish(self.stop_movement)
                print("Red Detected - Stopping")

            # Show image
            cv2.imshow('Camera Feed', image)
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")


def main():
    rclpy.init()
    robot = Robot()
    rclpy.spin(robot)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
