"""
формат сообщения:
    center: координаты центра
    direc: направление (forw/backw)
    size: расстояние между центрами синего и красного (чем больше, тем ближе)
    frame: разрешение кадра, с которого получена арука (w, h)
"""

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
        self.mode = 0 # if 1 that work, if 0 that debug
        self.debug_roi = 490, 380, 20, 20 # x,y,w,h

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera_standart/image_raw',
            self.process_image_callback,
            1
        )

        self.publisher = self.create_publisher(String, 'arrows', 2)

    def process_image_callback(self, msg):
        def detect_largest_region(mask):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None, None, 0
            
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            moments = cv2.moments(largest_contour)
            if moments["m00"] != 0:
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
            else:
                center_x, center_y = None, None
            return (center_x, center_y), area
        
        
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if self.mode == 0:
            x,y,w,h = self.debug_roi
            roi = hsv_frame[y:y+h, x:x+w]
            h_channel, s_channel, v_channel = cv2.split(roi)

            h_mean, h_min, h_max = np.mean(h_channel), np.min(h_channel), np.max(h_channel)
            s_mean, s_min, s_max = np.mean(s_channel), np.min(s_channel), np.max(s_channel)
            v_mean, v_min, v_max = np.mean(v_channel), np.min(v_channel), np.max(v_channel)

            print(f"H: mean={h_mean:.2f}, min={h_min}, max={h_max}")
            print(f"S: mean={s_mean:.2f}, min={s_min}, max={s_max}")
            print(f"V: mean={v_mean:.2f}, min={v_min}, max={v_max}")
            print()
            cv2.rectangle(frame,(x,y),(w,h),(0,255,0),3)
            cv2.imshow('Arrows_Debug', resize(2,frame))
            cv2.waitKey(1)
        else:
            arrows = {}
            #arrows['frame'] = (frame.shape[1], frame.shape[0])

            lab = cv2.cvtColor()
            


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
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()