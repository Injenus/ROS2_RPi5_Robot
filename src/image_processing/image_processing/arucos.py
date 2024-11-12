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

aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

class ArucoFinder(Node):
    def __init__(self):
        super().__init__('aruco_finder')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera_standart/image_raw',
            self.process_image_callback,
            1
        )

        self.publisher = self.create_publisher(String, 'arucos', 2)

    def process_image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        corners, ids, rej = cv2.aruco.detectMarkers(frame, aruco_dictionary)
        arucos = {}
        if ids is not None:
            arucos = {id[0]: tuple(corner[0]) for id, corner in zip(ids, corners)}

        json_message = String()
        json_message.data = json.dumps(arucos)
        self.publisher.publish(json_message)
        self.get_logger().info(f'Published JSON result: {json_message.data}')

def main(args=None):
    rclpy.init(args=args)
    image_processor = ArucoFinder()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()