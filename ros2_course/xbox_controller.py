import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import pygame
from std_msgs.msg import Float32MultiArray

# Avoid movement if the stick is moved slightly/stick drift
def apply_deadzone(value, threshold=0.2):
    return value if abs(value) > threshold else 0.0

class XBoxController(Node):
    
    def __init__(self):
        super().__init__('xbox_controller')
        
        
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/controller_cmd',
            10
        )

        self.timer = self.create_timer(0.05, self.timer_cb) # 20 Hz

        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    
    def timer_cb(self):
        pygame.event.pump()

        # Get values from sticks and left trigger
        left_x = apply_deadzone(-self.joystick.get_axis(0))
        left_y = apply_deadzone(self.joystick.get_axis(1))
        right_y = apply_deadzone(-self.joystick.get_axis(4))
        left_trigger = -self.joystick.get_axis(2)

        # Don't publish if idle
        if (left_x == 0.0 and left_y == 0.0 and right_y == 0.0 and left_trigger == 1.0):
            return

        msg = Float32MultiArray()
        msg.data = [left_x, left_y, right_y, left_trigger]
        print(msg)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    XBox = XBoxController()
    rclpy.spin(XBox)
    XBox.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
