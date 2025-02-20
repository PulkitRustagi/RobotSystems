from picarx_improved import Picarx
import logging
import time
import cv2
import time
import io
import numpy as np
from vilib import Vilib
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)


SAFE_DISTANCE = 10
DRIVE_SPEED = 30
FRAME_THRESHOLD = 60

class Sensing():
    def __init__(self, camera=False):
        self.px = Picarx()
        self.image_counter = 0
        if camera:
            print("Camera is on")
            self.px.set_cam_tilt_angle(-30)
            time.sleep(0.1)
            Vilib.camera_start(vflip=False,hflip=False)
            Vilib.display(local=True,web=True)
            self.name = 'img'
            self.path = f"picarx"
            time.sleep(0.5)
        else:
            print("Camera is off")
            exit(0)

    def get_camera_image(self):
        # function that gets a camera image
        Vilib.take_photo(self.name, self.path)
        # now save it in the current folder
        cv2.imwrite(f'{self.path}/{self.name}{self.image_counter}.png', Vilib.img)
        self.image_counter += 1

class Interpretation():
    def __init__(self, sensitivity=1, polarity=1):
        self.sensitivity = sensitivity
        self.polarity = polarity 
        self.image_id = 0

    def line_position_camera(self, image_path, image_name):
        """Takes camera data and uses OpenCV to convert to grayscale image, 
        thresholds to find line to follow, sets coordinate to -1 if line on 
        far left of screen, sets to 1 if on far right of screen"""
        camera_data = cv2.imread(f'{image_path}/{image_name}{self.image_id}.png')
        self.image_id += 1
        if camera_data is None:
            print("\n\ncamera data is None")
            exit(0)
            
        grayscale = cv2.cvtColor(camera_data, cv2.COLOR_BGR2GRAY)
        # Threshold
        if self.polarity == 1:
            # Lighter line
            _, binary = cv2.threshold(grayscale, 200, 255, cv2.THRESH_BINARY)  
        else:
            # Darker line
            _, binary = cv2.threshold(grayscale, 50, 255, cv2.THRESH_BINARY_INV) 

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0

        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        line_center = x + w / 2
        frame_center = camera_data.shape[1] / 2
        normalized_position = (line_center - frame_center) / frame_center
        print(f"Normalized position: {normalized_position}")

        return normalized_position
    
class Controller():
    def __init__(self, scaling_factor=30):
        self.angle_scale = scaling_factor
        
    def follow_line(self, px, line_position):
        if -0.1 < line_position < 0.1:
            px.set_dir_servo_angle(0)
            sonic_stop(px, sonic_sensor(px))
        else:
            px.set_dir_servo_angle(line_position*self.angle_scale)
            sonic_stop(px, sonic_sensor(px))


# ------------------------------------------------
# Ultrasonic Sensor Functions
# ------------------------------------------------
def sonic_sensor(px):
    """
    Reads the distance from the ultrasonic sensor.

    Returns:
        float: distance in centimeters, rounded to two decimals.
    """
    return round(px.ultrasonic.read(), 2)


def sonic_stop(px, distance):
    """
    Based on the distance from the ultrasonic sensor, either moves or stops the PiCar-X.

    Args:
        distance (float): The current distance reading from the ultrasonic sensor.
    """
    if distance < SAFE_DISTANCE:
        px.stop()
    else:
        px.forward(30)

if __name__ == "__main__":
    sensing = Sensing(camera=True)
    interpret = Interpretation(sensitivity=2.0, polarity=-1)
    controller = Controller(scaling_factor=30)

    while True:
        sensing.get_camera_image()  # stores the image in the path specified in the Sensing class "picarx/images"
        line_position = interpret.line_position_camera(sensing.path, sensing.name) # returns a float between -1 and 1
        controller.follow_line(sensing.px, line_position) # sets the direction of the servo and the speed of the car
        time.sleep(0.1)
        # break if intrerrupted
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    sensing.px.stop()
    Vilib.camera_stop()
    Vilib.display(local=False,web=False)
    logging.info("Exiting program")