from picamera2 import Picamera2
import cv2
import numpy as np
print('Run cpicam2')
class Rpi_Camera():
    def __init__(self, id, resolution, name='default_name', hard_resize_koeff=0.0, rotate=180, hard_roi=None, calib_data=None, gains_roi=(0,0,0.1,0.1)):
        assert resolution in [0,1,2,3] or isinstance(resolution, tuple)
        assert rotate in [0,90,180,270]
        self.cam = Picamera2(id)
        self.hard_roi = hard_roi
        
        if isinstance(resolution, tuple):
            w,h = self.hard_roi[2]-self.hard_roi[0], self.hard_roi[3]-self.hard_roi[1]
        elif resolution == 0:
            w,h = 2592, 1944
        elif resolution == 1:
            w,h = 1920, 1080
        elif resolution == 2:
            w,h = 1280, 720
        elif resolution == 3:
            w,h, = 640, 480
        self.w, self.h = w, h
        self.camera_config = self.cam.create_preview_configuration(main={"format": "RGB888", "size": (self.w, self.h)})
        self.cam.configure(self.camera_config)
        self.red_gain = 1.25
        self.blue_gain = 1.5
        self.gain = 15.0
        self.cam.set_controls({
                "AwbEnable": 0,  # Отключение автоматического баланса белого
                "ColourGains": (self.red_gain,self.blue_gain),
                "AeEnable": 0,  # Отключение автоматической экспозиции
                "AnalogueGain": self.gain,
                "ExposureTime": 33000  # Установка выдержки в микросекундах (например, 20000 = 20мс)
            })
        if self.hard_roi is not None:
            self.cam.set_controls({"ScalerCrop": self.hard_roi})
        self.name = name
        self.target_green = 160
        self.rotate = rotate
        self.frame = None
        self.undist_frame = None
        self.r_mean, self.g_mean, self.b_mean = None, None, None
        self.hard_resize_koeff = hard_resize_koeff
        self.gains_roi = [gains_roi[0]*self.w,gains_roi[1]*self.h, gains_roi[2]*self.w,gains_roi[3]*self.h]
        if self.hard_resize_koeff > 1.0:
            self.gains_roi = [round(el/self.hard_resize_koeff) for el in self.gains_roi]

        if calib_data is not None:
            self.cam_matrix = calib_data['camMatrix']
            self.dist_coeff = calib_data['distCoeff']
            self.cam_matrix_new_one = calib_data['camMatrixNew_one']
            self.cam_matrix_new_zero = calib_data['camMatrixNew_zero']
            self.rep_error = calib_data['repError']
            self.rvecs = calib_data['rvecs']
            self.tvecs = calib_data['tvecs']
            self.roi_one = calib_data['roi_one']
            self.roi_zero = calib_data['roi_zero']

        self.start()
    
    def start(self):
        try:
            self.cam.start()
        except:
            print('ERR start')

    def get_frame(self):
        if self.rotate == 90:
            self.frame = cv2.rotate(self.cam.capture_array(), cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate == 180:
            self.frame = cv2.rotate(self.cam.capture_array(), cv2.ROTATE_180)
        elif self.rotate == 270:
            self.frame = cv2.rotate(self.cam.capture_array(), cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        if self.hard_resize_koeff > 1.0:
            self.frame = self.hard_resize()
    
    def set_white_balance(self, manual_gains=None):
        self.cam.set_controls({"ColourGains": (self.red_gain, self.blue_gain),
                            "AnalogueGain": self.gain})

        #print(self.name,self.red_gain, self.blue_gain, self.gain)
    
    def adapt_color_coeff(self, input):
        return input*(0.01-0.001)/255.0+0.001

    def adapt_expos_coeff(self, input):
        return input*(1.0-0.02)/255.0+0.02

    def adjust_colour_gains(self, thresh_color=0.1, thresh_expos=0.1, manual_gains=None):
        if manual_gains is not None:
            assert isinstance(manual_gains, tuple) and len(manual_gains) == 3
            self.red_gain, self.blue_gain, self.gain = manual_gains
        else:
            # Корректируем красный и синий каналы, используя зеленый как эталон
            self.calculate_rgb_means()
            delta_r = abs(self.r_mean-self.g_mean)
            delta_b = abs(self.b_mean-self.g_mean)
            if delta_r > thresh_color:
                color_adj_factor = self.adapt_color_coeff(delta_r)
                if self.r_mean > self.g_mean:
                    self.red_gain -= color_adj_factor
                else:
                    self.red_gain += color_adj_factor

            if delta_b > thresh_color:
                color_adj_factor = self.adapt_color_coeff(delta_b)
                if self.b_mean > self.g_mean:
                    self.blue_gain -= color_adj_factor
                else:
                    self.blue_gain += color_adj_factor

            delta_green = abs(self.g_mean-self.target_green)
            if delta_green > thresh_expos:
                expos_adj_factor = self.adapt_expos_coeff(delta_green)
                if self.g_mean > self.target_green:
                    self.gain -= expos_adj_factor
                    if self.gain < 0.1:
                        self.gain = 0.1
                else:
                    self.gain += expos_adj_factor
                    if self.gain > 75:
                        self.gain = 75
        
        self.set_white_balance()

    def draw_gains_roi(self, color=(0,0,255)):
        #x, y, w, h = self.gains_roi
        drawed = self.frame.copy()
        drawed[self.gains_roi[1]:self.gains_roi[1]+self.gains_roi[3], self.gains_roi[0]:self.gains_roi[0]+self.gains_roi[2]] = color
        return drawed
    
    def central_roi(self, radius_size=60):
        #h,w,_ = self.frame.shape
        return self.frame[self.frame.shape[0]//2-radius_size:self.frame.shape[0]//2+radius_size, self.frame.shape[1]//2-radius_size:self.frame.shape[1]//2+radius_size, :]
    
    def calculate_rgb_means(self):
        # x, y, w, h = self.gains_roi
        # roi_frame = self.frame[y:y+h, x:x+w]
        roi_frame = self.frame[self.gains_roi[1]:self.gains_roi[1]+self.gains_roi[3], self.gains_roi[0]:self.gains_roi[0]+self.gains_roi[2]]
        #roi_frame = cv2.blur(roi_frame, (3,3))
        self.r_mean = np.mean(roi_frame[:, :, 2])
        self.g_mean = np.mean(roi_frame[:, :, 1])
        self.b_mean = np.mean(roi_frame[:, :, 0])

    def hard_resize(self):
        return cv2.resize(self.frame, (round(self.frame.shape[1]/self.hard_resize_koeff), round(self.frame.shape[0]/self.hard_resize_koeff)))
    
    def remove_distorsion(self, param=0):
        assert param in [0,1]
        if param == 0:
            x, y, w, h = self.roi_zero
            self.undist_frame = cv2.undistort(self.frame, self.cam_matrix, self.dist_coeff, None, self.cam_matrix_new_zero)
        elif param == 1:
            x, y, w, h = self.roi_one
            self.undist_frame = cv2.undistort(self.frame, self.cam_matrix, self.dist_coeff, None, self.cam_matrix_new_one)
        self.undist_frame = self.undist_frame[y:y+h, x:x+w]