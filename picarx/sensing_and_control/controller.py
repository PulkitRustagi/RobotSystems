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
    def __init__(self, max_turn_angle=30):
        self.max_turn_angle=max_turn_angle
        self.turn_servo = Servo("P2")

    def set_turn_proportion(self, turn_proportion):
        turn_angle = float(self.max_turn_angle*turn_proportion)
        self.turn_servo.angle(turn_angle)