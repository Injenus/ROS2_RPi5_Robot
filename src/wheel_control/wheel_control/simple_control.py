import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import serial
import time
import os
import sys

max_wheel_speed = 255

class SimpleWheelControl(Node):

    def __init__(self):
        super().__init__('simple_wheel_control')
        self.subscription = self.create_subscription(String,'vel/simple', self.listener_callback, 2)

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

    
    
    def listener_callback(self, msg):
        msg_dict = json.loads(msg.data)
        self.get_logger().info(f'Получено: {msg_dict}')

        def clip_val(val):
            if val < -max_wheel_speed:
                val = -max_wheel_speed
            elif val > max_wheel_speed:
                val = max_wheel_speed
            return val

        left_w, right_w = clip_val(round(msg_dict['l']*max_wheel_speed)), clip_val(round(msg_dict['r']*max_wheel_speed))

        checksum = left_w * 2 + right_w * 4
        data_to_send = f"s,{left_w},{right_w},{checksum},f"        

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(data_to_send.encode('utf-8'))
                self.get_logger().info(f'Отправлено в COM-порт: {data_to_send}')           
            except serial.SerialException as e:
                self.get_logger().error(f"Ошибка при отправке данных в COM-порт: {e}")    
        else:
            self.get_logger().info(f'Данные {data_to_send} не отправлены, COM-порт не открыт')

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("COM-порт закрыт")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    twist_subscriber = SimpleWheelControl()
    rclpy.spin(twist_subscriber)
    twist_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()