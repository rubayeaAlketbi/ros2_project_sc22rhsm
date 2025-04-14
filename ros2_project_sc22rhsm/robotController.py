from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.movement = Twist()

    def move_forward(self):
        self.movement.linear.x = 0.2
        self.publisher.publish(self.movement)

    def stop_robot(self):
        self.movement.linear.x = 0.0
        self.publisher.publish(self.movement)
