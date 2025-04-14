import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class ColourIdentifier(Node):
    def __init__(self):
        super().__init__('colour_identifier')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create a subscriber for the camera topic
        self.subscription = self.create_subscription(
            Image,                      # Message type
            'camera/image_raw',         # Topic name
            self.callback,              # Callback function
            10                          # Queue size
        )

    def callback(self, data):
        try:
            # Convert the ROS image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Display the image
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # Needed to update the image window

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")


def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    rclpy.init(args=None)
    cI = ColourIdentifier()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
