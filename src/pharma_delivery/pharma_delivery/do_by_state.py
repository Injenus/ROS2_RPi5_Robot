from tools import * 
from pid import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class DoByState(Node):
    def __init__(self):
        super().__init__('pharma_do_by_state')
        self.create_subscription(String, 'pharma_state', self.action, 2)

        self.publisher = self.create_publisher(String, 'vel/simple', 2)

        self.base_vel = 0.15
        self.min_vel = 0.1
        self.max_vel = 0.42

    def action(self, msg):
        self.state_data = json.loads(msg.data)

        if self.state_data['main_state'] == 0:
            pass



        self.get_logger().info(f'left={0}, right={0}')
        self.publisher.publish()


def main():
    rclpy.init(args=args)
    state_updater = DoByState()
    rclpy.spin(state_updater)
    state_updater.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()