from custom_picamera2 import *
from tools import *

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DualCameraPublisher(Node):
    def __init__(self):
        super().__init__('dual_camera_publisher')
        self.bridge = CvBridge()

        self.publisher_camera_standart = self.create_publisher(Image, 'camera_standart/image_raw', 1)
        self.publisher_camera_fisheye = self.create_publisher(Image, 'camera_fisheye/image_raw', 1)

        self.cam_standart = Rpi_Camera(id=1, resolution=0, name='standart', hard_resize_koeff=2.5, rotate=180, hard_roi=None, calib_data=None, gains_roi=(0,0,1,1)) # 1280 720    1920 1080     2592 1944
        self.cam_fisheye = Rpi_Camera(id=0, resolution=(1440,720), name='fisheye',hard_resize_koeff=5, rotate=180, hard_roi=(612,0,1332,1440), calib_data=None, gains_roi=(0.25,0.935,0.5,0.035))
        
        self.timer_standart = self.create_timer(0.08, self.publish_image_camera_standart)  # Публикация 10 FPS
        self.timer_fisheye = self.create_timer(0.08, self.publish_image_camera_fisheye)


    def publish_image_camera_standart(self):
        self.cam_standart.get_frame()
        self.cam_standart.adjust_colour_gains(manual_gains=(self.cam_fisheye.red_gain, self.cam_fisheye.blue_gain, self.cam_fisheye.gain))
        if self.cam_standart.frame is not None:
            self.cam_standart.frame = self.cam_standart.frame[50:, :, :]
            image_message = self.bridge.cv2_to_imgmsg(self.cam_standart.frame, encoding="bgr8")
            self.publisher_camera_standart.publish(image_message)
            self.get_logger().info('Published image from Camera_Standart')

    def publish_image_camera_fisheye(self):
        self.cam_fisheye.get_frame()
        self.cam_fisheye.adjust_colour_gains()
        self.get_logger().info(f'gain= {self.cam_fisheye.gain} , red= {self.cam_fisheye.red_gain} , blue= {self.cam_fisheye.blue_gain}')
        if self.cam_fisheye.frame is not None:
            self.cam_fisheye.frame = self.cam_fisheye.draw_gains_roi()
            image_message = self.bridge.cv2_to_imgmsg(cv2.cvtColor(self.cam_fisheye.frame, cv2.COLOR_BGR2GRAY), encoding="mono8")
            self.publisher_camera_fisheye.publish(image_message)
            self.get_logger().info('Published image from Camera_Fisheye')

def main(args=None):
    rclpy.init(args=args)
    dual_camera_publisher = DualCameraPublisher()
    rclpy.spin(dual_camera_publisher)
    dual_camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
