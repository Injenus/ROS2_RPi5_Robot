import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import keyboard
from .local_tools import *

base_linear_speed = 1.0/3
base_angular_speed = 1.0/3

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_talker')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 2)
        self.twist_msg = Twist()
        self.timer = self.create_timer(0.06, self.timer_callback)  # вызываем каждые 0.05 секунды

    def shutdown(self):
        self.get_logger().info('Завершение работы talker...')
        self.destroy_node()
        rclpy.shutdown()
        

    def timer_callback(self):
        def calc_speed_val(base_val, is_positive):
            speed = base_val + float(keyboard.is_pressed('v'))*base_val/2 + float(keyboard.is_pressed('shift'))*base_val
            if speed > 1.0:
                speed = 1.0
            if not is_positive:
                speed = -speed
            return speed

        if keyboard.is_pressed('e'):
            if keyboard.is_pressed('t'):
                self.get_logger().info('Инициировано заверешние работы...')
                self.shutdown()
        
        if keyboard.is_pressed('w'):
            self.twist_msg.linear.x = calc_speed_val(base_linear_speed, True)
        elif keyboard.is_pressed('s'):
            self.twist_msg.linear.x = calc_speed_val(base_linear_speed, False)
        else:
            self.twist_msg.linear.x = 0.0

        if keyboard.is_pressed('a'):
            self.twist_msg.angular.z = calc_speed_val(base_angular_speed, True)
        elif keyboard.is_pressed('d'):
            self.twist_msg.angular.z = calc_speed_val(base_angular_speed, False)
        else:
            self.twist_msg.angular.z = 0.0

        self.get_logger().info(f'x={self.twist_msg.linear.x}, z={self.twist_msg.angular.z}')

        self.publisher_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    rclpy.spin(keyboard_control)
    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

