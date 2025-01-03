"""
0 - ожидаем первичный аруко-маркер в центре, вращаясь против часовой
1 - ищем первую стрелку в аптеку
2 - двигаемся центрируясь по стрелке
3 - остановка перед стрлекой и поврот #на 90град.# согласно стрелке
4 - увидели вторичный арукомаркер
5 - двигаемся центриурясь по маркеру
6 - остновились перед марекром
7 - сдали чуть назад и разворот #на 180# (ищём первую стрелку из аптеки)
8 - развернулись от послденей стрелки
9 - насенсили финишную аруку

0 - едем по стрелкам в аптеку
1 - едем по стрелкам из аптеки
"""
from tools import * 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class StateUpdater(Node):
    def __init__(self):
        super().__init__('pharma_state_updater')
        self.states = {'arrow_state': 0, 'main_state': 0, 'turn': None, 'move_err': None}
        self.aruco_number = None
        self.relativ_thresh = 0.25
        self.backw_arrow_num = 6
        self.backw_counter = 0
        self.stop_size_arrow = 0.15
        self.stop_arrow_thresh = 0.1
        self.stop_size_aruco = 0.1
        self.error_scaller = 1000

        self.aruc_ofst = 0

        #self.aruco_offset =-0.1 * self.error_scaller

        self.create_subscription(String, 'arucos', self.aruco_callback, 2)
        self.create_subscription(String, 'arrow', self.arrow_callback, 2)

        self.publisher = self.create_publisher(String, 'pharma_state', 2)


    def aruco_callback(self, msg):
        data_dict = json.loads(msg.data)
        self.decide_next_action(aruco=data_dict)

    def arrow_callback(self, msg):
        data_dict = json.loads(msg.data)
        self.decide_next_action(arrows=data_dict)


    def decide_next_action(self, aruco=None, arrows=None):
        local_state = None # 0 если работаем с аруко, 1 - если со стрелкой
        if aruco is not None:
            assert isinstance(aruco, dict)
            if len(aruco) > 0:
                local_state = 0
        elif arrows is not None: 
            assert isinstance(arrows, dict)
            if len(arrows) > 0:
                local_state = 1
            # elif len(arrow) > 1:
            #     self.get_logger().info(f'Too many arrows {len(arrow)} during {self.states}')
        self.states['turn'] = None
        self.states['move_err'] = None

        if self.states['main_state'] == 0:
            if local_state == 0:
                if len(aruco) > 1:
                    self.get_logger().info(f'Too many arucos {len(aruco)} during {self.states}')
                else:
                    aruco = aruco['0']
                    self.aruco_number = aruco['num']
                    if 1-self.relativ_thresh <= aruco['center'][0]/(aruco['frame'][0]/2) <= 1+self.relativ_thresh:
                        self.states['main_state'] = 1
                    else:
                        pass
            elif local_state == 1:
                pass
        
        elif self.states['main_state'] == 1:
            if local_state == 1:
                for key in arrows.keys():
                    arr = arrows[key]
                    if 1-self.relativ_thresh <= arr['center'][0]/(arr['frame'][0]/2) <= 1+self.relativ_thresh:
                        if arr['type'] == 'forw':
                            self.states['main_state'] = 2
                            self.states['move_err'] = round(self.error_scaller*(arr['frame'][0]/2-arr['center'][0])/arr['frame'][0])
                            print(self.states['move_err'])
                        else:
                            self.get_logger().info(f'Sudden arrow {arr["type"]} during {self.states}')
   
        elif self.states['main_state'] == 2:
            if local_state == 1:
                for key in arrows.keys():
                    arr = arrows[key]
                    if arr['type'] == 'forw' and self.states['arrow_state'] == 0 or arr['type'] == 'backw' and self.states['arrow_state'] == 1:
                        if arr['size']/arr['frame'][0] > self.stop_size_arrow:
                            self.states['main_state'] = 3
                            self.states['turn'] = arr['direc']
                            self.states['move_err'] = 0
                            if self.states['arrow_state'] == 1:
                                self.backw_counter += 1
                        else:
                            self.get_logger().info(f'Moving to arrow..')
                            self.states['move_err'] = round(self.error_scaller*(arr['frame'][0]/2-arr['center'][0])/arr['frame'][0])
                    else:
                        self.get_logger().info(f'Ill-timed arrow')


        elif self.states['main_state'] == 3:
            if self.backw_counter == self.backw_arrow_num:
                self.states['main_state'] = 8
                self.get_logger().info(f'Last arrow now')
            else:
                if local_state == 0 and self.states['arrow_state'] == 0: #если увидели аруку выолняя поворот
                    valid_aruco_key = next((k for k, v in aruco.items() if v.get("num") == self.aruco_number), None)
                    if valid_aruco_key: # увидели нужную аруку
                        if self.states['arrow_state'] == 0:  # и ещё не были в аптеке
                            self.states['main_state'] = 4
                            self.states['move_err'] = round(self.error_scaller*(aruco[valid_aruco_key]['frame'][0]/2-max(aruco[valid_aruco_key]['center'][0]-self.aruc_ofst, 50))/aruco[valid_aruco_key]['frame'][0])
                            self.get_logger().info(f'View pharm aruco')
                        else: # если это на обратном пути
                            self.states['main_state'] = 8
                            self.get_logger().info(f'View finish aruco')
                    else:
                        pass # не увидели нужную аруку
                
                if local_state == 1: # если увидели стрелку выполняя поворот
                    for key in arrows.keys():
                        arr = arrows[key]
                        if arr['type'] == 'forw' and self.states['arrow_state'] == 0 or arr['type'] == 'backw' and self.states['arrow_state'] == 1 and arr['size']/arr['frame'][0] < self.stop_size_arrow-self.stop_arrow_thresh:
                            self.states['main_state'] = 2
                            self.get_logger().info(f'Start moving to arrow..')
                            self.states['move_err'] = round(self.error_scaller*(arr['frame'][0]/2-arr['center'][0])/arr['frame'][0])

        elif self.states['main_state'] == 4:
            if local_state == 0:
                valid_aruco_key = next((k for k, v in aruco.items() if v.get("num") == self.aruco_number), None)
                if valid_aruco_key: # увидели нужную аруку
                    self.states['main_state'] = 5
                    self.states['arrow_state'] = 1
                    self.states['move_err'] = round(self.error_scaller*(aruco[valid_aruco_key]['frame'][0]/2-max(aruco[valid_aruco_key]['center'][0]-self.aruc_ofst, 50))/aruco[valid_aruco_key]['frame'][0])
                    self.get_logger().info(f'Viewing aruco')

        elif self.states['main_state'] == 5:
            if local_state == 0:
                valid_aruco_key = next((k for k, v in aruco.items() if v.get("num") == self.aruco_number), None)
                if valid_aruco_key: # увидели нужную аруку
                    self.states['move_err'] = round(self.error_scaller*(aruco[valid_aruco_key]['frame'][0]/2-max(aruco[valid_aruco_key]['center'][0]-self.aruc_ofst, 50))/aruco[valid_aruco_key]['frame'][0])
                    self.get_logger().info(f'Moving to aruco')

                    if aruco[valid_aruco_key]['size']/aruco[valid_aruco_key]['frame'][0] > self.stop_size_aruco:
                        self.states['main_state'] = 6
                        self.states['move_err'] = 0
                        self.get_logger().info(f'Stop to aruco')

        elif self.states['main_state'] == 6:
            self.get_logger().info(f'Wait for grab')
            self.states['main_state'] = 7
            self.get_logger().info(f'Return and 180 turn')

        elif self.states['main_state'] == 7:
            if local_state == 1:
                for key in arrows.keys():
                    arr = arrows[key]
                    if arr['type'] == 'backw':
                        self.states['main_state'] = 2

        elif self.states['main_state'] == 8:
            if local_state == 0:
                valid_aruco_key = next((k for k, v in aruco.items() if v.get("num") == self.aruco_number), None)
                if valid_aruco_key: # увидели нужную аруку
                    self.get_logger().info(f'Viewed finish aruco')
                    self.states['main_state'] == 9
                else:
                    self.get_logger().info(f'Viewed not finish aruco(s): {aruco}')


        json_message = String()
        json_message.data = json.dumps(self.states)
        self.publisher.publish(json_message)
        self.get_logger().info(f'Published JSON result: {json_message.data}')


def main(args=None):
    rclpy.init(args=args)
    state_updater = StateUpdater()
    rclpy.spin(state_updater)
    state_updater.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

