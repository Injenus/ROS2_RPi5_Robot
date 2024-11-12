from tools import * 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json

class ArrowFinder(Node):
    def __init__(self):
        super().__init__('arrow_finder')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera_standart/image_raw',
            self.process_image_callback,
            1
        )

        self.publisher = self.create_publisher(String, 'arrows', 2)

    def process_image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        arrows = {}
        


        json_message = String()
        json_message.data = json.dumps(arrows)
        self.publisher.publish(json_message)
        self.get_logger().info(f'Published arrows: {json_message.data}')

def main(args=None):
    rclpy.init(args=args)
    image_processor = ArrowFinder()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()