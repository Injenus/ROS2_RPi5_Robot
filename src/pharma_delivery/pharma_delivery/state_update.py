"""
0 - ожидаем первичный аруко-маркер в центре, вращаясь против часовой
1 - ищем первую стрелку в аптеку
2 - двигаемся центрируясь по стрелке
3 - остановка перед стрлекой и поврот #на 90град.# согласно стрелке
4 - увидели вторичный арукомаркер
5 - двигаемся центриурясь по маркеру
6 - остновились перед марекром
7 - сдали чуть назад и разворот #на 180# (ищём первую стрелку из аптеки)
8 - заметили аруку
9 - едем чуть чуть вслепую
10 - разворот на 90 на аруку
11 - финиш

0 - едем по стрелкам в аптеку
1 - едем по стрелкам из аптеки
"""
from tools import * 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class StateUpdater(Node):
    def __init__(self):
        super().__init__('pharma_state_updater')
        self.states = {'arrow_state': 0, 'main_state': 0}
        self.aruco_number = None
        self.relativ_thresh = 0.25
        
        self.create_subscription(String, 'arucos', self.aruco_callback, 2)
        self.create_subscription(String, 'arrows', self.arrow_callback, 2)

        self.publisher = self.create_publisher(String, 'pharma_state', 2)


    def aruco_callback(self, msg):
        data_dict = json.loads(msg.data)
        self.decide_next_action(aruco=data_dict)

    def arrow_callback(self, msg):
        data_dict = json.loads(msg.data)
        self.decide_next_action(arrow=data_dict)


    def decide_next_action(self, aruco=None, arrow=None):
        local_state = None # 0 если работаем с аруко, 1 - если со стрелкой
        if aruco is not None:
            assert isinstance(aruco, dict)
            local_state = 0
        elif arrow is not None:
            assert isinstance(arrow, dict)
            local_state = 1

        if self.states['main_state'] == 0:
            if local_state == 0:
                if len(aruco) > 1:
                    self.get_logger().info(f'Too many arucos {len(aruco)} during {self.states}')
                else:
                    aruco = aruco[0]
                    self.aruco_number = aruco['num']
                    if 1-self.relativ_thresh <= aruco['center'][0]/aruco['frame'][0]/2 <= 1+self.relativ_thresh:
                        self.states['main_state'] = 1
                    else:
                        pass
            elif local_state == 1:
                pass
        
        if self.states['main_state'] == 1:
            if local_state == 0:
                pass
            elif local_state == 1:
                if len(arrow) > 1:
                    self.get_logger().info(f'Too many arrows {len(arrow)} during {self.states}')
                else:
                    if 1-self.relativ_thresh <= arrow['center'][0]/arrow['frame'][0]/2 <= 1+self.relativ_thresh:
                        if arrow['direc'] == 'forw':
                            self.states['main_state'] = 2
                        else:
                            self.get_logger().info(f'Sudden arrow {arrow['direc']} during {self.states}')
                    else:
                        pass

        if self.states['main_state'] == 2:
            pass


        


        json_message = String()
        json_message.data = json.dumps(self.states)
        self.publisher.publish(json_message.data)
        self.get_logger().info(f'Published JSON result: {json_message.data}')


def main(args=None):
    rclpy.init(args=args)
    state_updater = StateUpdater()
    rclpy.spin(state_updater)
    state_updater.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

