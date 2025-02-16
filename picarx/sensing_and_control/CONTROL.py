from robot_hat import ADC
from robot_hat import Grayscale_Module


class Sensor():

    def __init__(self, Picar):
        self.px = Picar

    def sensor_read(self):
        sensor_values = self.px.get_grayscale_data()
        return sensor_values
    

class Interpretter():

    def __init__(self, px, sensitivity = 150, polarity = 1, reference = [500,500,500]):
        self.px = px
        self.sensitivity = sensitivity
        self.polarity = polarity
        self.reference = self.px.set_grayscale_reference(reference)
        self.ref = reference[0]

    def process(self, sensor_val_list):
        sensitivity = self.sensitivity
        Left, Middle, Right = sensor_val_list
        diff1 = Middle - Left
        diff2 = Right - Middle
        position = None
        if abs(diff1) > sensitivity and abs(diff2) > sensitivity and diff1 < 0:
            # picar is at the center
            position = 0
        elif abs(diff1) > sensitivity and abs(diff2) < sensitivity:
            if diff1 > 0:
                # Slight right
                position = -0.5
            elif diff1 < 0:
                # Sharp left
                position = +1 
        elif abs(diff1) < sensitivity and abs(diff2) > sensitivity:
            if diff2 > 0:
                # Sharp right
                position = -1
            elif diff2 < 0:
                # Slight left
                position = 0.5
        else:
            position = 0 
        return position 
    

class Controller():

    def __init__(self, Picar, scale_factor = 30):
        self.scale_factor = -scale_factor
        self.px = Picar

    def align_steering(self,offset):
        servo_angle = offset*self.scale_factor
        self.px.set_dir_servo_angle(servo_angle)