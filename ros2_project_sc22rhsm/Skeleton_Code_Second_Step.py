# Exercise 2 - detecting two colours, and filtering out the third colour and background.



import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal



class colourIdentifier(Node):
    def __init__(self):
        super().__init__('colour_identifier')

        # Initialise sensitivity for colour detection (10 should be sufficient)
        self.sensitivity = 10

        # Initialise CvBridge
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )

        self.subscription  # prevent unused variable warning

    def callback(self, data):
        try:
            
            # Convert the received image into a opencv image
            # But remember that you should always wrap a call to this conversion method in an exception handler
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
            cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', image)
            cv2.resizeWindow('camera_Feed',320,240)
            cv2.waitKey(3)
            
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
            # Set the upper and lower bounds for the two colours you wish to identify
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

            mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            res_green = cv2.bitwise_and(image, image, mask=mask_green)
            cv2.imshow('green', res_green)
            cv2.imshow('Green Filtered Image', res_green)
            cv2.waitKey(3)
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")
            
def main():
    
    def signal_handler(sig, frame):
        rclpy.shutdown()

    
    # Instantiate your class
    
    # And rclpy.init the entire node
    rclpy.init(args=None)
    cI = colourIdentifier()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass
    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()
    
# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
