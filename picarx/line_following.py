import os
import sys
import time

path = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(path, "..")
sys.path.append(path)

from picarx_improved import Picarx
from sensing_and_control.CONTROL import Sensor
from sensing_and_control.CONTROL import Interpretter
from sensing_and_control.CONTROL import Controller

def line_following(picar):
    sensor = Sensor(picar)
    interpretter = Interpretter(picar)
    controller = Controller(picar)


    while(True):
        picar.forward(30)
        data = sensor.sensor_read()
        turn_proportion = interpretter.process(data)
        print("Set till here 1")
        controller.align_steering(turn_proportion)
        print("Set till here 2")
        time.sleep(0.1)


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

