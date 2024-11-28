"""
формат сообщения:
    center: координаты центра
    type: направление (forw/backw)
    direc: направление (r/l)
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
import numpy as np

class ArrowFinder(Node):
    def __init__(self):
        super().__init__('arrow_finder')
        self.is_debug = bool(self.declare_parameter('is_debug', 0).value)
        self.is_show = bool(self.declare_parameter('is_show', 0).value)

        self.debug_roi = 490, 380, 42, 42 # x,y,w,h

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera_standart/image_raw',
            self.process_image_callback,
            1
        )

        self.publisher = self.create_publisher(String, 'arrow', 2)

        self.red_mask_0 = Color_Mask_HSV(170,181,128,255,64,255)
        self.red_mask_1 = Color_Mask_HSV(-1,10,128,255,64,255)
        self.blue_mask = Color_Mask_HSV(97,117,128,255,64,255)

        self.dist_equal = 8400

        # self.red_mask_lab = Color_Mask_LAB(168,189,168,210,207,233)
        # self.blue_mask_lab = Color_Mask_LAB(96,118,236,255,185,220)


    def process_image_callback(self, msg):
        
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.blur(frame,(5,5))
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

        if self.is_debug:
            x,y,w,h = self.debug_roi

            hsv_roi = hsv_frame[y:y+h, x:x+w]
            h_channel, s_channel, v_channel = cv2.split(hsv_roi)
            h_mean, h_min, h_max = np.mean(h_channel), np.min(h_channel), np.max(h_channel)
            s_mean, s_min, s_max = np.mean(s_channel), np.min(s_channel), np.max(s_channel)
            v_mean, v_min, v_max = np.mean(v_channel), np.min(v_channel), np.max(v_channel)
            print(f"H: mean={h_mean:.2f}, min={h_min}, max={h_max}")
            print(f"S: mean={s_mean:.2f}, min={s_min}, max={s_max}")
            print(f"V: mean={v_mean:.2f}, min={v_min}, max={v_max}")
            print()

            lab_roi = lab_frame[y:y+h, x:x+w]
            l_channel, a_channel, b_channel = cv2.split(lab_roi)
            l_mean, l_min, l_max = np.mean(l_channel), np.min(l_channel), np.max(l_channel)
            a_mean, a_min, a_max = np.mean(a_channel), np.min(a_channel), np.max(a_channel)
            b_mean, b_min, b_max = np.mean(b_channel), np.min(b_channel), np.max(b_channel)
            print(f"L: mean={l_mean:.2f}, min={l_min}, max={l_max}")
            print(f"A: mean={a_mean:.2f}, min={a_min}, max={a_max}")
            print(f"B: mean={b_mean:.2f}, min={b_min}, max={b_max}")
            print()

            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3)
            cv2.imshow('Arrows_Debug', resize(2,frame))
            cv2.waitKey(1)
        else:
            def morph(mask):
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)), iterations=1)
                # mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)), iterations=1)
                # mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)))
                # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT,(7,7)))
                # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)))
                return mask
            arrows = {}
            
            red_mask_0 = cv2.inRange(hsv_frame, np.array(self.red_mask_0.mins), np.array(self.red_mask_0.maxs))
            red_mask_1 = cv2.inRange(hsv_frame, np.array(self.red_mask_1.mins), np.array(self.red_mask_1.maxs))
            red_mask = red_mask_0 + red_mask_1
            blue_mask = cv2.inRange(hsv_frame, np.array(self.blue_mask.mins), np.array(self.blue_mask.maxs))

            red_mask = morph(red_mask)
            blue_mask = morph(blue_mask)
            
            if self.is_show:
                r_b = np.dstack((red_mask, np.zeros_like(red_mask), blue_mask))
                cv2.imshow('r_b', resize(2, cv2.cvtColor(r_b, cv2.COLOR_RGB2BGR)))
                cv2.waitKey(1)
                
            red_roi_shapes = detect_shapes(red_mask)
            blue_roi_shapes = detect_shapes(blue_mask)

            def check_for_arrow(red_roi, blue_roi, type_direc): #forw or backw
                assert type_direc in {'forw', 'backw'}
                print(red_roi, blue_roi)
                r_c, b_c = red_roi[3], blue_roi[3]
                is_exsist = (red_roi[0] > -1 and blue_roi[0] > -1)
                is_near = (((r_c[0]-b_c[0])**2+(r_c[1]-b_c[1])**2)**0.5 <= (r_c[2]+b_c[2]) or ((r_c[0]-b_c[0])**2+(r_c[1]-b_c[1])**2)**0.5 - (r_c[2]+b_c[2]) < self.dist_equal)
                if is_exsist and is_near:
                    if type_direc == 'forw':
                        return blue_roi[2] > red_roi[2]
                    elif type_direc == 'backw':
                        return red_roi[2] > blue_roi[2]

            
            forw_check = check_for_arrow(red_roi_shapes[0], blue_roi_shapes[1], "forw")
            backw_check = check_for_arrow(red_roi_shapes[1], blue_roi_shapes[0], 'backw')

            def send_arrow_data(red_roi, blue_roi, condit):
                if condit:
                    if abs(red_roi[1] - blue_roi[1])/frame.shape[0] < 0.25: # если по высоте примерно одинаково
                        size = round(((red_roi[0]-blue_roi[0])**2+(red_roi[1]-blue_roi[1])**2)**0.5)
                        if size/frame.shape[1] < 1.0:
                            arrow = {}
                            arrow['center'] = (round((red_roi[0]+blue_roi[0])/2), round((red_roi[1]+blue_roi[1])/2))
                            arrow['size'] = size
                            arrow['frame'] = (frame.shape[1], frame.shape[0]) # w, h
                            if blue_roi[2] > red_roi[2]:
                                arrow['type'] = 'forw'
                                if blue_roi[0] < red_roi[0]:
                                    arrow['direc'] = 'l'
                                elif red_roi[0] < blue_roi[0]:
                                    arrow['direc'] = 'r'
                                else:
                                    arrow['direc'] = None
                            elif red_roi[2] > blue_roi[2]:
                                arrow['type'] = 'backw'
                                if blue_roi[0] < red_roi[0]:
                                    arrow['direc'] = 'r'
                                elif red_roi[0] < blue_roi[0]:
                                    arrow['direc'] = 'l'
                                else:
                                    arrow['direc'] = None
                            else:
                                arrow['type'] = None
                                arrow['direc'] = None

                            arrows[0] = arrow

                            if self.is_show:
                                cv2.circle(frame, (red_roi[0], red_roi[1]), 9, (0,160,255), thickness=-1)
                                cv2.circle(frame, (blue_roi[0], blue_roi[1]), 9, (255,160,0), thickness=-1)

                                # cv2.circle(frame, (r_c[0], r_c[1]), r_c[2], (0,160,255), thickness=3)
                                # cv2.circle(frame, (b_c[0], b_c[1]), b_c[2], (255,160,0), thickness=3)
                
                                cv2.circle(frame, arrow['center'], 9, (0,255,0), thickness=-1)
                                cv2.imshow('Arrows', resize(2, frame))
                                cv2.waitKey(1)
                            
                        else:
                            self.get_logger().info(f'Too long! {size/frame.shape[1]}')
                    else:
                        self.get_logger().info(f'Too diffrent heights! {abs(red_roi[1] - blue_roi[1])/frame.shape[0]}')


                    json_message = String()
                    json_message.data = json.dumps(arrows)
                    self.publisher.publish(json_message)
                    self.get_logger().info(f'Published arrows: {json_message.data}')
                else:
                    self.get_logger().info(f'Not exsist ot not near!')

            send_arrow_data(red_roi_shapes[0], blue_roi_shapes[1], forw_check)
            send_arrow_data(red_roi_shapes[1], blue_roi_shapes[0], backw_check)

           

def main(args=None):
    rclpy.init(args=args)
    image_processor = ArrowFinder()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()