import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from picarx_improved import Picarx
from time import sleep

class Sensor:
    def __init__(self):
        self.px = Picarx()
        self.px.set_line_reference([1000, 1000, 1000])

    def read(self):
        return self.px.get_grayscale_data()

class Interpreter:
    def __init__(self, sensitivity=500, polarity=1):
        """
        sensitivity: the minimum difference between adjacent pin values required to detect an edge.
        polarity: +1 if the line is darker than the surrounding floor
                  -1 if the line is lighter than the surrounding floor
        """
        self.sensitivity = sensitivity
        self.polarity = polarity

    def process(self, sensor_data):
        """
        Identifies the position of the robot relative to the line using sensor differences.

        sensor_data: [left_pin_value, center_pin_value, right_pin_value],
                     each in the range [0, 4096].

        Returns a float in [-1, 1] indicating:
          +1   : sharp left
          +0.5 : slight left
           0   : center / no turn
          -0.5 : slight right
          -1   : sharp right
           None: line not detected (all differences below sensitivity threshold)
        """
        # Unpack sensor readings
        left_raw, center_raw, right_raw = sensor_data

        # Apply polarity: if polarity=1 (line is darker), invert readings to standardize processing
        left_val = left_raw
        center_val = center_raw
        right_val = right_raw

        # Compute differences between adjacent sensors
        diff_left_center = left_val - center_val
        diff_center_left = center_val - left_val
        diff_right_center = right_val - center_val
        diff_center_right = center_val - right_val

        # Determine pattern based on differences exceeding sensitivity
        # if diff_left_center > self.sensitivity:
        #     pattern = [1, 0, 0]  # Sharp left
        #     return 'sharp left'
        # elif diff_center_right > self.sensitivity:
        #     pattern = [1, 1, 0]  # Slight left
        #     return 'slight left'
        # elif diff_right_center > self.sensitivity:
        #     pattern = [0, 0, 1]  # Sharp right
        #     return 'sharp right'
        # elif diff_center_left > self.sensitivity:
        #     pattern = [0, 1, 1]  # Slight right
        #     return 'slight right'
        # elif diff_center_left > self.sensitivity and diff_center_right > self.sensitivity:
        #     pattern = [0, 1, 0]  # Centered
        #     return 'centered'
        # else:
        #     # No significant difference detected between any adjacent sensors
        #     return pass  # Keep previous action (line is out of range)

        if left_val - center_val > self.sensitivity and right_val - center_val > self.sensitivity:
            return 'centered'
        elif center_val - left_val > self.sensitivity and right_val - left_val > self.sensitivity:
            return 'sharp left'
        elif center_val - left_val < self.sensitivity and right_val - left_val > self.sensitivity:
            return 'slight left'
        elif left_val - center_val > self.sensitivity and left_val - right_val > self.sensitivity:
            return 'sharp right'
        elif left_val - center_val < self.sensitivity and left_val - right_val > self.sensitivity:
            return 'slight right'
        else:
            return 'no line detected'
            
    def output(self, processed_data):
        if processed_data == 'sharp left':
            return -1.0
        elif processed_data == 'slight left':
            return -0.5
        elif processed_data == 'sharp right':
            return 1.0
        elif processed_data == 'slight right':
            return 0.5
        elif processed_data == 'centered':
            return 0.0
        else:
            return 0.0
    
class Controller:
    def __init__(self, scale=10):
        self.scale = scale

    def control(self, offset):
        angle = offset * self.scale
        return angle

def main():
    sensor = Sensor()
    interpreter = Interpreter(sensitivity=40)
    controller = Controller(scale=30)

    while True:
        sensor_data = sensor.read()
        print(f"Sensor data: {sensor_data}")
        processed_data = interpreter.process(sensor_data)
        print(f"Line following maneuver needer: {processed_data}")
        output_data = interpreter.output(processed_data)
        angle = controller.control(output_data)
        print(f"Changing servo angle to: {angle} to follow the line\n----------------")
        sensor.px.move_discrete('forward', 30, angle=angle, duration=0.2)
        # sleep(0.1)
    
main()