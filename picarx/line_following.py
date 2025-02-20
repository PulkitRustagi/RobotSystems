import os
import sys
import time

path = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(path, "..")
sys.path.append(path)

from picarx_improved import Picarx
from sensing_and_control.sensor import Sensor
from sensing_and_control.interpreter import Interpreter
from sensing_and_control.controller import Controller

def line_following(picar):
    sensor = Sensor()
    interpreter = Interpreter()
    controller = Controller(picar)

    try:
        while(True):
            picar.forward(30)
            data = sensor.read_data()
            turn_proportion = interpreter.interpret_sensor_reading_PID(data, k_p=0.3, k_i=0.001, k_d=0.02)
            print("Set till here 1")
            controller.align_steering(turn_proportion)
            print("Set till here 2")
            time.sleep(0.05)
    except:
        pass

def main():
    px = Picarx()
    # px.set_cam_tilt_angle(0)
    # px.set_cam_tilt_angle(-30)
    line_following(px)
    print("Set till here 3")
    px.stop()
    print("Set till here 4")

if __name__ == "__main__":

    main()

