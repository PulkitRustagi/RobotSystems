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

class Sensor():
    def __init__(self):
        # set adc structure using self.syntax
        self.adc = [ADC(0), ADC(1), ADC(2)]

        self.rotation_index = 0
        self.num_samples = 3
        self.num_sensors = 3
        self.grayscale_data = np.zeros((self.num_samples, self.num_sensors))

        for i in range(self.num_samples):
            for j in range(self.num_sensors):
                self.grayscale_data[i, j] = self.adc[j].read()

    
    def read_data(self):
        # get grayscale values
        for i in range(self.num_sensors):
            self.grayscale_data[self.rotation_index, i] = self.adc[i].read()

        self.rotation_index += 1
        self.rotation_index %= self.num_samples

        data = np.mean(self.grayscale_data, axis=0)
        print("mean data from grayscale: ", data)
        # return values
        return(data)
        