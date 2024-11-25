import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import os
import sys
from .local_tools import *

max_wheel_speed = 255

class TwistSubscriber(Node):

    def __init__(self):
        super().__init__('com_listener')
        self.subscription = self.create_subscription(Twist,'cmd_vel', self.listener_callback, 2)
        self.subscription  # предотвращает предупреждение об удалении

        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                timeout=1
            )
            self.get_logger().info("COM-порт успешно открыт")
        except serial.SerialException as e:
            self.get_logger().error(f"Ошибка при открытии COM-порта: {e}")
            self.serial_port = None

        
        self.check_timer = self.create_timer(1.0, self.check_activity)
        self.timeout_duration = 5.0  # время в секундах
        self.last_msg_time = time.time() 

    def check_activity(self):
        if time.time() - self.last_msg_time > self.timeout_duration:
            self.get_logger().warning('Сообщения не поступали, перезапуск узла...')
            self.restart_node()

    def restart_node(self):
        # Завершаем ROS2
        rclpy.shutdown()
        # Перезапускаем процесс узла
        os.execv(sys.executable, ['python3'] + sys.argv)
    
    def listener_callback(self, msg):
        self.last_msg_time = time.time() 
        self.get_logger().info(f'Get: linear x: {msg.linear.x}, angular z: {msg.angular.z}')

        left_w, right_w = 0, 0

        if msg.linear.x == 0:
            if msg.angular.z < 0:
                left_w = -round(max_wheel_speed*msg.angular.z)
                right_w = -left_w
            elif msg.angular.z > 0:
                right_w = round(max_wheel_speed*msg.angular.z)
                left_w = -right_w
        elif msg.linear.x > 0:
            if msg.angular.z < 0:
                left_w = -round(max_wheel_speed*msg.angular.z)
                right_w = 0
            elif msg.angular.z > 0:
                right_w = round(max_wheel_speed*msg.angular.z)
                left_w = 0
            else:
                right_w = round(max_wheel_speed*msg.linear.x)
                left_w = right_w
        else:
            if msg.angular.z < 0:
                left_w = round(max_wheel_speed*msg.angular.z)
                right_w = 0
            elif msg.angular.z > 0:
                right_w = -round(max_wheel_speed*msg.angular.z)
                left_w = 0
            else:
                right_w = round(max_wheel_speed*msg.linear.x)
                left_w = right_w
    
        checksum = left_w * 2 + right_w * 4
        data_to_send = f"s,{left_w},{right_w},{checksum},f"        

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(data_to_send.encode('utf-8'))
                self.get_logger().info(f'Sended: {data_to_send}')           
            except serial.SerialException as e:
                self.get_logger().error(f"Error sending: {e}")    
        else:
            self.get_logger().info(f'Data {data_to_send} didnt send, COM port is close')

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("COM port closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    twist_subscriber = TwistSubscriber()
    rclpy.spin(twist_subscriber)
    twist_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
