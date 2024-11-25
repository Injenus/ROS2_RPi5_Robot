"""
формат сообщения:
    словарь линий:
        center: координаты центра х,у (центр масс)
        line_area: площадь самой линии в пикселях
        rect_area: площадь рамки вокруг линии в пикселях
        orient: ориентация линии hor/ver/unk
        frame: разрешение кадра w,h

"""
from tools import * 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json

class LineFinder(Node):
    def __init__(self):
        super().__init__('line_finder')
        self.is_debug = bool(self.declare_parameter('is_debug', 0).value)

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera_fisheye/image_raw',
            self.process_image_callback,
            1
        )

        self.publisher = self.create_publisher(String, 'lines', 1)

    def process_image_callback(self, msg):
        
        def calculate_area_manual(binary_image, contour):
            mask = np.zeros_like(binary_image, dtype=np.uint8)
            cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
            area = np.sum(mask == 255)
            return area
        
        lines = {}
        gray_frame = self.bridge.imgmsg_to_cv2(msg, "mono8")
        _, binary_frame = cv2.threshold(gray_frame, 191, 255, cv2.THRESH_BINARY)

        height, width = binary_frame.shape

        black_start = int(height * (1-0.2))
        binary_frame[black_start:, :] = 0 
        binary_frame[0, :] = 0  # Верхняя граница
        binary_frame[-1, :] = 0  # Нижняя граница
        binary_frame[:, 0] = 0  # Левая граница
        binary_frame[:, -1] = 0  # Правая граница

        contours, _ = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
     
        if self.is_debug:
            output_image = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)
            font = cv2.FONT_HERSHEY_SIMPLEX

        for i, contour in enumerate(contours):
            local_dict = {}
            area = calculate_area_manual(binary_frame, contour)
            moments = cv2.moments(contour)
            if moments['m00'] != 0:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
            else:
                cx, cy = None, None
            
            x, y, w, h = cv2.boundingRect(contour)
            bounding_area = w * h

            local_dict['center'] = (cx,cy)
            local_dict['line_area'] = area
            local_dict['rect_area'] = bounding_area
            local_dict['orient'] = 'unk'
            local_dict['frame'] = (binary_frame.shape[1], binary_frame.shape[0])
            if w > 1.33*h:
                local_dict['orient'] = 'hor'
            elif h > 1.33*w:
                local_dict['orient'] = 'ver'
            lines[i] = local_dict

            if self.is_debug:
                cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.circle(output_image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(output_image, f'Area: {int(area)}', (x, y - 10), font, 0.5, (0, 255, 0), 1)
                cv2.putText(output_image, f'BBox: {int(bounding_area)}', (x, y + h + 15), font, 0.5, (0, 0, 255), 1)

        if self.is_debug:
            cv2.imshow('line_debug', output_image)
            cv2.waitKey(1)

        json_message = String()
        json_message.data = json.dumps(lines)
        self.publisher.publish(json_message)
        self.get_logger().info(f'Published lines: {json_message.data}')

def main(args=None):
    rclpy.init(args=args)
    image_processor = LineFinder()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__=='__main__':
    main()