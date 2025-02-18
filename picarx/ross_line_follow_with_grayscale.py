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

SAFE_DISTANCE = 50
DRIVE_SPEED = 30
FRAME_THRESHOLD = 60

def line_following_with_obstacle(picar):
    sensor = Sensor()
    interpreter = Interpreter()
    controller = Controller(picar)

    try:
        while(True):
            picar.forward(30)
            data = sensor.read_data()
            turn_proportion = interpreter.interpret_sensor_reading_PID(data, k_p=0.3, k_i=0.001, k_d=0.02)
            controller.set_turn_proportion(turn_proportion)
            sonic_stop(picar, sonic_sensor(picar))
            time.sleep(0.05)
    except:
        pass

# ------------------------------------------------
# Ultrasonic Sensor Functions
# ------------------------------------------------
def sonic_sensor(picar):
    """
    Reads the distance from the ultrasonic sensor.

    Returns:
        float: distance in centimeters, rounded to two decimals.
    """
    return round(picar.ultrasonic.read(), 2)


def sonic_stop(picar, distance):
    """
    Based on the distance from the ultrasonic sensor, either moves or stops the PiCar-X.

    Args:
        distance (float): The current distance reading from the ultrasonic sensor.
    """
    if distance < SAFE_DISTANCE:
        picar.stop()
    else:
        picar.forward(DRIVE_SPEED)

def main():
    px = Picarx()
    px.set_cam_tilt_angle(0)
    px.set_cam_tilt_angle(-30)
    line_following_with_obstacle(px)
    px.stop()

if __name__ == "__main__":

    main()

