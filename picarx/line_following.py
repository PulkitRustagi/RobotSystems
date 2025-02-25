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
    interpretter = Interpreter()
    controller = Controller(picar)


    while(True):
        picar.forward(30)
        data = sensor.sensor_read()
        turn_proportion = interpretter.process(data)
        controller.align_steering(turn_proportion)
        time.sleep(0.1)


def main():
    px = Picarx()
    line_following(px)
    px.stop()

if __name__ == "__main__":
    main()

