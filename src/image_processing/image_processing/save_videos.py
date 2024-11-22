from tools import *

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
import datetime

saving_path = '../../../rpi_robot_py_modules'
class DualCameraSubscriber(Node):
    def __init__(self):
        super().__init__('dual_camera_subscriber')
        self.bridge = CvBridge()

        self.subscription_camera_standart = self.create_subscription(
            Image,
            'camera_standart/image_raw',
            self.display_and_save_camera_standart,
            1
        )
        
        self.subscription_camera_fisheye = self.create_subscription(
            Image,
            'camera_fisheye/image_raw',
            self.display_and_save_camera_fisheye,
            1
        )
        
        now = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
        self.video_path_standart = os.path.join(saving_path, f'camera_standart_video_{now}.avi')
        self.video_path_fisheye = os.path.join(saving_path, f'camera_fisheye_video_{now}.avi')
        print(self.video_path_standart)
        print(self.video_path_fisheye)

  
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.frame_rate = 10  # FPS записи

    
        self.out_standart = None
        self.out_fisheye = None

        self.timer_st = time.time()
        self.timer_fe = time.time()

    def display_and_save_camera_standart(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        
        if self.out_standart is None:
            height, width = frame.shape[:2]
            self.out_standart = cv2.VideoWriter(self.video_path_standart, self.fourcc, self.frame_rate, (width, height))

        
        self.out_standart.write(frame)
        
        
        cv2.imshow('Camera_Standart', resize(1.5, frame))
        cv2.waitKey(1)
        print(f'st {1 / (time.time() - self.timer_st)}')
        self.timer_st = time.time()

    def display_and_save_camera_fisheye(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        
        if self.out_fisheye is None:
            height, width = frame.shape[:2]
            self.out_fisheye = cv2.VideoWriter(self.video_path_fisheye, self.fourcc, self.frame_rate, (width, height))

       
        self.out_fisheye.write(frame)
        
        
        cv2.imshow('Camera_Fisheye', resize(1.5, frame))
        cv2.waitKey(1)
        print(f'fe {1 / (time.time() - self.timer_fe)}')
        self.timer_fe = time.time()

    def destroy_node(self):
        if self.out_standart:
            self.out_standart.release()
        if self.out_fisheye:
            self.out_fisheye.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    dual_camera_subscriber = DualCameraSubscriber()
    rclpy.spin(dual_camera_subscriber)
    dual_camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
