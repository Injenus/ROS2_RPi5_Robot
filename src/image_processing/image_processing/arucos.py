"""
формат сообщения:
{
    num: номер
    #coord: координаты стандартные (углов)
    center: координаты центра
    size: средняя длина стороны
    frame: разрешение кадра, с которого получена арука (w, h)
}
    ...
"""
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
            for i, corner in enumerate(corners):
                local_aruco= {}
                pts = corner[0]  # массив формы (4, 2), содержащий x, y углов
             
                center_x = np.mean(pts[:, 0]).item(0)
                center_y = np.mean(pts[:, 1]).item(0)
                
                side_lengths = [np.linalg.norm(pts[j] - pts[(j + 1) % 4]) for j in range(4)]

                local_aruco['num'] = ids[i][0].item(0)
                local_aruco['center'] = (round(max(center_x-170, 50)), round(center_y))
                local_aruco['size'] = round(sum(side_lengths)/4, 2)
                local_aruco['frame'] = (frame.shape[1], frame.shape[0]) # w, h
                arucos[i] = local_aruco

                if ids[i][0].item(0) == 5:
                    cv2.circle(frame, (round(center_x), round(center_y)), 10, (0,0,255), -1)
                    cv2.imshow('aruco', resize(3,frame))
                    cv2.putText(frame, str(round(local_aruco['size']/local_aruco['frame'][0], 2)), local_aruco['center'],cv2.FONT_HERSHEY_SIMPLEX, 3, (0,255,0), 2)
                    cv2.waitKey(1)

        json_message = String()
        json_message.data = json.dumps(arucos)
        self.publisher.publish(json_message)
        self.get_logger().info(f'Published JSON result: {json_message.data}')

def main(args=None):
    rclpy.init(args=args)
    image_processor = ArucoFinder()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()