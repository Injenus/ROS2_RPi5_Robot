import sys
util_path = "/home/inj/rpi_robot_py_modules/"
sys.path.append(util_path)
from tools import * 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json

class TrafficLightFinder(Node):
    def __init__(self):
        super().__init__('traffic_light_finder')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera_standart/image_raw',
            self.process_image_callback,
            1
        )

        self.publisher = self.create_publisher(String, 'traffic_lights', 2)

    def process_image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        traffic_lights = {}
        


        json_message = String()
        json_message.data = json.dumps(traffic_lights)
        self.publisher.publish(json_message)
        self.get_logger().info(f'Published traffic_lights: {json_message.data}')

def main(args=None):
    rclpy.init(args=args)
    image_processor = TrafficLightFinder()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()