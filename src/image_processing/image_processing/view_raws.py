from tools import *

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class DualCameraSubscriber(Node):
    def __init__(self):
        super().__init__('dual_camera_subscriber')
        self.bridge = CvBridge()

        self.subscription_camera_standart = self.create_subscription(
            Image,
            'camera_standart/image_raw',
            self.display_image_camera_standart,
            1
        )
        
        self.subscription_camera_fisheye = self.create_subscription(
            Image,
            'camera_fisheye/image_raw',
            self.display_image_camera_fisheye,
            1
        )
        self.timer_st = time.time()
        self.timer_fe = time.time()

    def display_image_camera_standart(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('Camera_Standart', resize(2,frame))
        cv2.waitKey(1)
        print(f'st {1/(time.time()-self.timer_st)}')
        self.timer_st = time.time()

    def display_image_camera_fisheye(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('Camera_Fisheye', frame)
        cv2.waitKey(1)
        print(f'fe {1/(time.time()-self.timer_fe)}')
        self.timer_fe = time.time()

def main(args=None):
    rclpy.init(args=args)
    dual_camera_subscriber = DualCameraSubscriber()
    rclpy.spin(dual_camera_subscriber)
    dual_camera_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
