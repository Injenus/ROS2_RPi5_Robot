import cv2
import numpy as np

def resize(koeff, frame):
    assert frame is not None
    return cv2.resize(frame, (round(frame.shape[1]/koeff), round(frame.shape[0]/koeff)))