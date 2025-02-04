from pydoc import text
from vilib import Vilib
from time import sleep, time, strftime, localtime
# import geom_util as geom
import threading
import readchar
import os
import cv2 as cv
import numpy as np
import roi

Roi = roi.ROI()

flag_face = False
flag_color = False
qr_code_flag = False
## Picture settings

# initial grayscale threshold
threshold = 120

# max grayscale threshold
threshold_max = 180

#min grayscale threshold
threshold_min = 40

# iterations to find balanced threshold
th_iterations = 10

# min % of white in roi
white_min=3

# max % of white in roi
white_max=12

## Driving settings

# line angle to make a turn
turn_angle = 45

# line shift to make an adjustment
shift_max = 20

# turning time of shift adjustment
shift_step = 0.125

# turning time of turn
turn_step = 0.25

# time of straight run
straight_run = 0.5

# attempts to find the line if lost
find_turn_attempts = 5

# turn step to find the line if lost
find_turn_step = 0.2

# max # of iterations of the whole tracking
max_steps = 40

# target brightness level
brightness = 100

def take_photo():
    _time = strftime('%Y-%m-%d-%H-%M-%S',localtime(time()))
    name = 'photo_%s'%_time
    username = os.getlogin()

    path = f"/home/{username}/Pictures/"
    Vilib.take_photo(name, path)
    print('photo save as %s%s.jpg'%(path,name))



def find_main_countour(image):
    im2, cnts, hierarchy = cv.findContours(image, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
    C = None
    if cnts is not None and len(cnts) > 0:
         C = max(cnts, key = cv.contourArea)
    if C is None:
        return None, None
    rect = cv.minAreaRect(C)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    box = order_box(box)
    return C, box


def order_box(box):
    srt = np.argsort(box[:, 1])
    btm1 = box[srt[0]]
    btm2 = box[srt[1]]

    top1 = box[srt[2]]
    top2 = box[srt[3]]

    bc = btm1[0] < btm2[0]
    btm_l = btm1 if bc else btm2
    btm_r = btm2 if bc else btm1

    tc = top1[0] < top2[0]
    top_l = top1 if tc else top2
    top_r = top2 if tc else top1

    return np.array([top_l, top_r, btm_r, btm_l])

def balance_pic(image):
    global T
    ret = None
    direction = 0
    for i in range(0, th_iterations):
        rc, gray = cv.threshold(image, T, 255, 0)
        crop = Roi.crop_roi(gray)
        nwh = cv.countNonZero(crop)
        perc = int(100 * nwh / Roi.get_area())
        if perc > white_max:
            if T > threshold_max:
                break
            if direction == -1:
                ret = crop
                break
            T += 10
            direction = 1
        elif perc < white_min:
            if T < threshold_min:
                break
            if  direction == 1:
                ret = crop
                break
            T -= 10
            direction = -1
        else:
            ret = crop
            break  
    return ret      