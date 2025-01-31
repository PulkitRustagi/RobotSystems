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
    # Instantiate sensor, interpreter, and controller objects
    sensor = Sensor()
    interpreter = Interpreter()
    controller = Controller()

    try:
        while(True):
            picar.forward(30)
            data = sensor.read_data()
            turn_proportion = interpreter.interpret_sensor_reading_PID(data, k_p=0.3, k_i=0.001, k_d=0.02)
            
            controller.set_turn_proportion(turn_proportion)
            time.sleep(0.05)
    except:
        pass

def main():
    # Instantiate your Picarx object
    px = Picarx()
    line_following(px)
    px.stop()

if __name__ == "__main__":

    main()

