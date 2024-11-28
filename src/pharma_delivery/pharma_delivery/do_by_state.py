from tools import * 
from delta_pid import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random

class DoByState(Node):
    def __init__(self):
        super().__init__('pharma_do_by_state')
        self.create_subscription(String, 'pharma_state', self.action, 2)

        self.publisher = self.create_publisher(String, 'vel/simple', 2)

        self.base_vel = 0.33
        self.min_vel = 0.22
        self.max_vel = 0.5

        self.turn_90_time = 1.15
        self.stop_time = 0.33
        self.backw_time = 3.0
        self.one_cell_time = 3.5
        self.dance_time = 0.05
        self.forw_time = 3.0

        self.start_aruco = True
        self.find_arrow = False
        self.turn_flag = False
        self.stop_phrama_flag = True
        self.reverse_flag = True
        self.finish_flag = True

        self.vels = {'l': 0, 'r': 0}

        self.pid = PID(0.001, 0.1, 0.01, 0.0, 0.0)
        self.init_t = time.time()

    def action(self, msg):
        def control(err):
            return simple_ctrl(err)

        self.dt = time.time() - self.init_t
        self.init_t = time.time()
        self.state_data = json.loads(msg.data)

        if self.state_data['main_state'] == 0:
            self.vels['l'] = -self.min_vel*0.85 # вращение против часовой
            self.vels['r'] = self.min_vel*0.85
            self.publish_msg(False)

        elif self.state_data['main_state'] == 1:
            if self.start_aruco:
                self.stop_immediately()
                time.sleep(self.stop_time)
                self.start_aruco = False
            self.vels['l'] = -self.min_vel*0.85
            self.vels['r'] = self.min_vel*0.85
            self.publish_msg(False)

        elif self.state_data['main_state'] == 2:
            if not self.find_arrow:
                self.stop_immediately()
                time.sleep(self.stop_time)
                self.find_arrow = True
            if self.state_data['move_err']:
                self.turn_flag = True
                #correct_val = self.pid.calculate(self.state_data['move_err'], self.dt) # если ошибка больше 0 то надо ускорить правое, замедлить левое
                correct_val = control(self.state_data['move_err'])
                self.check_init_vels()
                self.vels['l'] -= correct_val                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
                self.vels['r'] += correct_val
                self.vels['l'] = max(self.vels['l'], self.min_vel)
                self.vels['r'] = max(self.vels['r'], self.min_vel)
                self.publish_msg()
            else:
                # self.stop_immediately()
                # self.dance()
                self.slow_move() 
            

        elif self.state_data['main_state'] == 3:
            
            if self.find_arrow:
                self.find_arrow = False

            if self.turn_flag:
                self.stop_immediately()
                self.pid.reset()

                if self.state_data['turn'] == 'l':
                    self.turn_left()
                elif self.state_data['turn'] == 'r':
                    self.turn_right()
                else:
                    self.stop_immediately()
                    self.dance()
                    
                time.sleep(self.stop_time)
                self.turn_flag = False # поворот только после движения к стрелке и один раз
                self.publish_msg()


        elif self.state_data['main_state'] == 4 or self.state_data['main_state'] == 5:
            if self.state_data['move_err']:
                #correct_val = self.pid.calculate(self.state_data['move_err'], self.dt)
                correct_val = control(self.state_data['move_err'])
                self.check_init_vels()
                self.vels['l'] -= correct_val
                self.vels['r'] += correct_val
                self.vels['l'] = max(self.vels['l'], self.min_vel)
                self.vels['r'] = max(self.vels['r'], self.min_vel)
            else:
                self.dance()
            self.publish_msg()

        elif self.state_data['main_state'] == 6:
            if self.stop_phrama_flag:
                self.stop_immediately()
                self.pid.reset()
                time.sleep(self.stop_time)
                self.stop_phrama_flag = False
                self.turn_flag = True

        elif self.state_data['main_state'] == 7:
            if self.reverse_flag:
                self.stop_immediately()
                self.move_forward()
                self.stop_immediately()
                self.move_backward()
                self.turn_right()
                self.stop_immediately()
                self.turn_right()
                self.reverse_flag = False
                self.turn_flag = True

        elif self.state_data['main_state'] == 8:
            if self.finish_flag:
                self.move_one_cell()
                self.turn_left()
                self.finish_flag = False

        elif self.state_data['main_state'] == 9:
            self.get_logger().info(f'FINISH')




    def check_init_vels(self):
        if self.vels['l'] in {-self.min_vel, 0}:
            self.vels['l'] = self.base_vel
        if self.vels['r'] in {self.min_vel, 0}:
            self.vels['r'] = self.base_vel

    def publish_msg(self, is_clip = True):
        
        def clip(val):
            if val < self.min_vel:
                val = self.min_vel
            if val > self.max_vel:
                val = self.max_vel
            return val
        if is_clip:
            self.vels['l'] = clip(self.vels['l'])
            self.vels['r'] = clip(self.vels['r'])
        json_message = String()
        json_message.data = json.dumps(self.vels)
        self.publisher.publish(json_message)
        self.get_logger().info(f'Published JSON result: {json_message.data}')

    def turn_left(self):
        self.vels['l'] = -self.base_vel
        self.vels['r'] = self.base_vel
        self.publish_msg(False)
        time.sleep(self.turn_90_time)
        self.stop_immediately()

    def turn_right(self):
        self.vels['l'] = self.base_vel
        self.vels['r'] = -self.base_vel
        self.publish_msg(False)
        time.sleep(self.turn_90_time)
        self.stop_immediately()

    def stop_immediately(self):
        self.vels['l'] = 0
        self.vels['r'] = 0
        self.publish_msg(False)

    def move_backward(self):
        self.vels['l'] = -self.min_vel
        self.vels['r'] = -self.min_vel
        self.publish_msg(False)
        time.sleep(self.backw_time)
        self.stop_immediately()

    def move_forward(self):
        self.vels['l'] = self.min_vel
        self.vels['r'] = self.min_vel
        self.publish_msg(False)
        time.sleep(self.forw_time)
        self.stop_immediately()

    def move_one_cell(self):
        self.vels['l'] = self.min_vel
        self.vels['r'] = self.min_vel
        self.publish_msg()
        self.vels['l'] = self.base_vel
        self.vels['r'] = self.base_vel
        self.publish_msg()
        time.sleep(self.one_cell_time)
        self.vels['l'] = self.min_vel
        self.vels['r'] = self.min_vel
        self.publish_msg()
        self.stop_immediately()

    def dance(self):
        for i in range(random.randint(1, 5)):
            if i % 2:
                self.vels['l'] = self.base_vel*1.5
                self.vels['r'] = -self.base_vel*1.5
            else:
                self.vels['l'] = -self.base_vel*1.5
                self.vels['r'] = self.base_vel*1.5
            self.publish_msg(False)
            time.sleep(self.dance_time)
        self.stop_immediately()

    def slow_move(self):
        self.vels['l'] = 0.5*self.min_vel
        self.vels['r'] = 0.5*self.min_vel
        self.publish_msg()



def main(args=None):
    rclpy.init(args=args)
    state_updater = DoByState()
    rclpy.spin(state_updater)
    state_updater.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()