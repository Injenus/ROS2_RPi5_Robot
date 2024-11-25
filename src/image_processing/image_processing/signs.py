"""
формат сообщений:
    для каждого знака:
        center:
"""
from tools import * 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json

class SignFinder(Node):
    def __init__(self):
        super().__init__('sign_finder')
        #self.is_debug = bool(self.declare_parameter('is_debug', 0).value)
        self.is_show = bool(self.declare_parameter('is_show', 0).value)

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera_standart/image_raw',
            self.process_image_callback,
            1
        )

        self.publisher = self.create_publisher(String, 'signs', 2)

        self.blue_mask = Color_Mask_HSV(100,200,100,200,100,200)
        self.red_mask = Color_Mask_HSV(100,200,100,200,100,200)

    def process_image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        signs = {}

        red_mask = cv2.inRange(hsv_frame, np.array(self.red_mask.mins), np.array(self.red_mask.maxs))
        blue_mask = cv2.inRange(hsv_frame, np.array(self.blue_mask.mins), np.array(self.blue_mask.maxs))



        


        json_message = String()
        json_message.data = json.dumps(signs)
        self.publisher.publish(json_message)
        self.get_logger().info(f'Published signs: {json_message.data}')

def main(args=None):
    rclpy.init(args=args)
    image_processor = SignFinder()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()