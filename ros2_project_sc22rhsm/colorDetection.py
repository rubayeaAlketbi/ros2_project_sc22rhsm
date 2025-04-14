import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node

class ColourDetector(Node):
    def __init__(self):
        super().__init__('colour_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.callback, 10)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Define color ranges
            hsv_blue = (np.array([100, 150, 50]), np.array([130, 255, 255]))

            # Create color masks
            mask_blue = cv2.inRange(hsv_image, hsv_blue[0], hsv_blue[1])

            # Find contours
            contours, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 500:
                    print("Blue Box Detected!")

            cv2.imshow("Camera Feed", image)
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")

def main():
    rclpy.init()
    node = ColourDetector()
    rclpy.spin(node)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
