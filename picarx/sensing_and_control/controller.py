import time
import numpy as np

try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic
    from robot_hat.utils import reset_mcu, run_command
    on_robot = True
    reset_mcu()

except ImportError:
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic
    on_robot = False

class Controller():
    def __init__(self, Picar, scale_factor = 30):
        self.scale_factor = -scale_factor
        self.px = Picar

    def align_steering(self, offset):
        servo_angle = offset*self.scale_factor
        self.px.set_dir_servo_angle(servo_angle)
