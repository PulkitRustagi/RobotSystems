import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
# print("path: ", sys.path[-1]) 
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
        sensitivity: integer threshold for deciding if a pin is 'on' or 'off'
        polarity: +1 if the line is darker than surrounding floor
                  -1 if the line is lighter than surrounding floor
        """
        self.sensitivity = sensitivity
        self.polarity = polarity

    def process(self, sensor_data):
        """
        sensor_data: [left_pin_value, center_pin_value, right_pin_value],
                     each in the range [0, 4096].
        
        Returns a float in [-1, 1] indicating:
          +1   : sharp left
          +0.5 : slight left
           0   : center / no turn
          -0.5 : slight right
          -1   : sharp right
           None: no sensor pins detect the line (line is out of range)
        """
        # Unpack sensor readings
        left_raw, center_raw, right_raw = sensor_data
        
        # If polarity=1, the line is darker => raw readings are smaller on the line.
        # Invert so that a darker (lower) reading becomes a larger "processed" value.
        if self.polarity == 1:
            left_val = 4096 - left_raw
            center_val = 4096 - center_raw
            right_val = 4096 - right_raw
        else:
            # polarity = -1 => the line is lighter => raw readings are larger on the line.
            # Keep them as-is (or adjust as needed).
            left_val = left_raw
            center_val = center_raw
            right_val = right_raw

        # Decide if each pin qualifies as "1" (on the line) or "0" (off the line)
        # based on the sensitivity threshold.
        left_on = 1 if left_val >= self.sensitivity else 0
        center_on = 1 if center_val >= self.sensitivity else 0
        right_on = 1 if right_val >= self.sensitivity else 0

        # Compare against the patterns you listed:
        pattern = [left_on, center_on, right_on]

        if pattern == [1, 0, 0]:
            # Sharp left
            return 1.0
        elif pattern == [1, 1, 0]:
            # Slight left
            return 0.5
        elif pattern == [0, 1, 0]:
            # Centered
            return 0.0
        elif pattern == [0, 1, 1]:
            # Slight right
            return -0.5
        elif pattern == [0, 0, 1]:
            # Sharp right
            return -1.0
        elif pattern == [0, 0, 0]:
            # Line is not detected by any pin => keep going or handle accordingly
            return None
        else:
            # If you wish to handle additional edge cases or overlapping conditions,
            # you can specify more logic here. By default, treat them as "center."
            return 0.0