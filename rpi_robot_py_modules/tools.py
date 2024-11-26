import cv2
import numpy as np

def resize(koeff, frame):
    assert frame is not None
    return cv2.resize(frame, (round(frame.shape[1]/koeff), round(frame.shape[0]/koeff)))

def calculate_area_manual(binary_image, contour):
    mask = np.zeros_like(binary_image, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
    area = np.sum(mask == 255)
    return area


class Color_Mask_HSV():
    def __init__(self, h_min, h_max, s_min, s_max, v_min, v_max):
        self.h_min = h_min
        self.h_max = h_max
        self.s_min = s_min
        self.s_max = s_max
        self.v_min = v_min
        self.v_max = v_max
        self.mins = [self.h_min, self.s_min, self.v_min]
        self.maxs = [self.h_max, self.s_max, self.v_max]


class Color_Mask_LAB():
    def __init__(self, l_min, l_max, a_min, a_max, b_min, b_max):
        self.l_min = l_min
        self.l_max = l_max
        self.a_min = a_min
        self.a_max = a_max
        self.b_min = b_min
        self.b_max = b_max
        self.mins = [self.l_min, self.a_min, self.b_min]
        self.maxs = [self.l_max, self.a_max, self.b_max]