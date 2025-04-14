import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class AutonomousRobot(Node):
    def __init__(self):
        super().__init__('autonomous_robot')

        self.bridge = CvBridge()
        self.sensitivity = 10  

        self.focal_length = 800
        self.real_object_width = 0.2
        self.stop_distance = 1.0
        self.stop_tolerance = 0.05

        self.blue_detected = False
        self.red_detected = False
        self.green_detected = False

        self.blue_distance = None
        self.obstacle_detected = False
        self.blue_centered = False
        self.exploring = True
        self.stuck_counter = 0

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_subscription = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', 10)

        self.timer = self.create_timer(2.0, self.exploration_behavior)

    def calculate_distance(self, contour_area):
        if contour_area > 0:
            estimated_distance = (self.real_object_width * self.focal_length) / np.sqrt(contour_area)
            return round(estimated_distance, 2)
        return None

    def process_color(self, image, hsv_image, color_range, draw_color, label):
        mask = cv2.inRange(hsv_image, color_range[0], color_range[1])
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        estimated_distance = None
        object_center_x = None

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(largest_contour)

            if contour_area > 500:
                estimated_distance = self.calculate_distance(contour_area)
                x, y, w, h = cv2.boundingRect(largest_contour)
                object_center_x = x + w // 2

                cv2.drawContours(image, [largest_contour], -1, draw_color, 2)
                cv2.rectangle(image, (x, y), (x + w, y + h), draw_color, 2)
                cv2.putText(image, f"{label}: {estimated_distance}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)
                cv2.circle(image, (object_center_x, y + h // 2), 5, draw_color, -1)

        return image, mask, estimated_distance, object_center_x

    def image_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            image = cv2.resize(image, (400, 300))
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            hsv_blue = (np.array([110, 100, 100]), np.array([130, 255, 255]))
            hsv_red = (np.array([0, 100, 100]), np.array([10, 255, 255]))
            hsv_green = (np.array([50, 100, 100]), np.array([70, 255, 255]))

            annotated = image.copy()

            annotated, mask_blue, dist_blue, blue_x = self.process_color(
                annotated, hsv_image, hsv_blue, (255, 0, 0), "Blue")
            annotated, mask_red, dist_red, _ = self.process_color(
                annotated, hsv_image, hsv_red, (0, 0, 255), "Red")
            annotated, mask_green, dist_green, _ = self.process_color(
                annotated, hsv_image, hsv_green, (0, 255, 0), "Green")

            self.blue_detected = dist_blue is not None
            self.red_detected = dist_red is not None
            self.green_detected = dist_green is not None

            # Show message if no detection
            if not (self.blue_detected or self.red_detected or self.green_detected):
                cv2.putText(annotated, "No colors detected", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

            frame_center_x = image.shape[1] // 2
            tolerance = 100
            if blue_x:
                if abs(blue_x - frame_center_x) > tolerance:
                    self.blue_centered = False
                    if blue_x < frame_center_x:
                        self.rotate_left()
                        print("🔄 Rotating left to center blue box")
                    else:
                        self.rotate_right()
                        print("🔄 Rotating right to center blue box")
                else:
                    self.blue_centered = True
                    print("✅ Blue box centered!")

            cv2.imshow("Detection & Navigation", annotated)
            cv2.waitKey(3)

            if self.blue_detected and self.blue_centered:
                self.blue_distance = dist_blue
                print(f"🔵 Blue Box - Distance: {dist_blue} meters")
                self.exploring = False
                self.navigate_to_blue()
            else:
                self.exploring = True

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")

    def navigate_to_blue(self):
        if self.blue_detected and self.blue_centered and not self.obstacle_detected:
            cmd = Twist()
            lower_bound = self.stop_distance - self.stop_tolerance
            upper_bound = self.stop_distance + self.stop_tolerance
            if lower_bound < self.blue_distance < upper_bound:
                cmd.linear.x = 0.0
                print("🛑 Reached blue box within tolerance. Stopping.")
            elif self.blue_distance > self.stop_distance:
                cmd.linear.x = 0.2
                print("➡️ Moving forward toward blue box...")
            else:
                cmd.linear.x = 0.0
                print("🛑 Stopping - close to blue box")
            self.cmd_vel_publisher.publish(cmd)

    def exploration_behavior(self):
        if self.exploring and not self.obstacle_detected:
            cmd = Twist()
            cmd.linear.x = 0.15
            cmd.angular.z = np.random.uniform(-0.5, 0.5)
            print("🌍 Exploring environment...")
            self.cmd_vel_publisher.publish(cmd)
        elif self.obstacle_detected:
            print("⏸️ Exploration paused due to obstacle.")

    def lidar_callback(self, data):
        front_angles = list(data.ranges[0:15]) + list(data.ranges[-15:])
        min_distance = min(front_angles)
        if min_distance < 0.4:
            self.obstacle_detected = True
            print("⚠️ Obstacle detected in front, executing avoidance maneuver...")
            self.avoid_obstacle()
        else:
            if self.obstacle_detected:
                print("✅ Obstacle cleared, resuming exploration.")
            self.obstacle_detected = False
            self.stuck_counter = 0

    def odom_callback(self, data):
        self.current_position = data.pose.pose.position

    def avoid_obstacle(self):
        self.stuck_counter += 1
        cmd = Twist()
        if self.stuck_counter > 3:
            cmd.linear.x = -0.1
            cmd.angular.z = 0.6
            print("🔁 Backing up and turning to unstuck...")
            self.stuck_counter = 0
        else:
            cmd.angular.z = 0.6
            print("↩️ Turning to avoid obstacle...")
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(1.0)
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

    def rotate_left(self):
        cmd = Twist()
        cmd.angular.z = 0.2
        self.cmd_vel_publisher.publish(cmd)

    def rotate_right(self):
        
        cmd = Twist()
        cmd.angular.z = -0.2
        self.cmd_vel_publisher.publish(cmd)


def main():
    rclpy.init()
    node = AutonomousRobot()
    rclpy.spin(node)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
