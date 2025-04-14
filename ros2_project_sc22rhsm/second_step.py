import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ColourIdentifier(Node):
    def __init__(self):
        super().__init__('colour_identifier')

        # Sensitivity for color detection
        self.sensitivity = 10
        self.bridge = CvBridge()

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

            hsv_blue_lower = np.array([110, 100, 100])
            hsv_blue_upper = np.array([130, 255, 255])

            # Create masks
            mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            mask_red1 = cv2.inRange(hsv_image, hsv_red_lower1, hsv_red_upper1)
            mask_red2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            mask_blue = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

            # Combine masks
            combined_mask = cv2.bitwise_or(mask_green, cv2.bitwise_or(mask_red, mask_blue))

            # Apply mask
            result = cv2.bitwise_and(image, image, mask=combined_mask)

            # Find contours for each color
            self.find_and_draw_contours(image, mask_green, (0, 255, 0))  # Green
            self.find_and_draw_contours(image, mask_red, (0, 0, 255))    # Red
            self.find_and_draw_contours(image, mask_blue, (255, 0, 0))   # Blue

            # Show results
            cv2.imshow('Original Image', image)
            cv2.imshow('Filtered Colors', result)
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")

    def find_and_draw_contours(self, image, mask, color):
        """Finds contours and draws bounding boxes on detected objects."""
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for c in contours:
            if cv2.contourArea(c) > 500:  # Ignore small noise
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)

                # Draw enclosing circle
                (cx, cy), radius = cv2.minEnclosingCircle(c)
                cv2.circle(image, (int(cx), int(cy)), int(radius), color, 2)


def main():
    rclpy.init()
    node = ColourIdentifier()
    rclpy.spin(node)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
