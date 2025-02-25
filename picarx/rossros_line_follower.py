"""
Camera-Based Line Follower with Obstacle Detection (Ultrasonic Sensor)

This script initializes a PiCar-X, reads camera frames to detect a line,
and steers accordingly. It also reads from an ultrasonic sensor to stop
the car if an obstacle is detected within a safe distance.

References:
1. OpenCV Documentation: https://docs.opencv.org/
2. Python Logging Documentation: https://docs.python.org/3/howto/logging.html
3. SunFounder PiCar-X Documentation: https://www.sunfounder.com/learn/
4. rossros Library: https://pypi.org/project/rossros/
"""

import time
import logging
import cv2
import numpy as np
from vilib import Vilib
from picarx_improved import Picarx
from sensing_and_control.CONTROL import Controller as CONTROLLER
import rossros as rr 

logging.getLogger().setLevel(logging.INFO)

# Global objects and constants
px = Picarx()
SAFE_DISTANCE = 20
DRIVE_SPEED = 30
FRAME_THRESHOLD = 60

def sensor_cam():
    """
    Captures a single image from the camera and returns it as a NumPy array.
    """
    
    time.sleep(0.1)
    Vilib.camera_start(vflip=False,hflip=False)
    Vilib.display(local=True,web=True)
    time.sleep(0.2)

    t = 1
    name = f"image{t}"
    path = "picarx/"
    status = Vilib.take_photo(name, path)

    if status and Vilib.img is not None and isinstance(Vilib.img, np.ndarray):
        full_path = f"{path}/{name}.jpg"
        cv2.imwrite(full_path, Vilib.img)
        captured_frame = cv2.imread(full_path)
        logging.warning(f"Image {name} has been captured by camera sensor module.")
        return captured_frame
    else:
        logging.warning("Failed to capture a valid image from camera.")
        return None


def interpretter_cam(frame):
    """
    Processes the captured frame to find the largest contour (presumed to be the line).
    Calculates the x-ratio with respect to the image center.

    Returns:
        float: x_ratio in the range [-1, 1], representing how far off-center the line is.
    """
    if frame is None:
        logging.warning("No frame available from sensor.")
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, FRAME_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

    frame_height, frame_width = frame.shape[:2]
    lower_half = binary[frame_height // 2:, :]

    contours, _ = cv2.findContours(lower_half, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        logging.warning("No contours found in frame.")
        return None

    center_line = max(contours, key=cv2.contourArea)
    centroid = cv2.moments(center_line)

    if centroid['m00'] == 0:
        logging.warning("No valid centroid found in the largest contour.")
        return None

    x_center = int(centroid['m10'] / centroid['m00'])
    x_ratio = (x_center - frame_width / 2) / (frame_width / 2)
    return x_ratio


def angle_controller(x_ratio):
    """
    Adjusts the steering angle of the PiCar-X based on the x_ratio of the line center.

    Args:
        x_ratio (float): Value in [-1, 1] representing how far off-center the line is.
    """
    if x_ratio is None:
        return

    # Negate to adjust direction if necessary
    adjustment = -1 * x_ratio
    control = CONTROLLER(px, scale_factor=30)
    control.align_steering(adjustment)


# ------------------------------------------------
# Ultrasonic Sensor Functions
# ------------------------------------------------
def sonic_sensor_reading(px):
    """
    Reads the distance from the ultrasonic sensor.

    Returns:
        float: distance in centimeters, rounded to two decimals.
    """
    distance = px.get_distance()
    while distance < 0:
        distance = px.ultrasonic.read()
    print(f"Distance from sonar: {distance}")
    return distance


def sonic_stop(px, distance):
    """
    Based on the distance from the ultrasonic sensor, either moves or stops the PiCar-X.

    Args:
        distance (float): The current distance reading from the ultrasonic sensor.
    """
    print(f"Distance: {distance}")
    if distance < SAFE_DISTANCE:
        px.stop()
    else:
        px.forward(30)


# ------------------------------------------------
# Main Execution (Concurrently)
# ------------------------------------------------
def main():
    """
    Sets up concurrency tasks for:
      1. Camera capture and line interpretation
      2. Angle control
      3. Ultrasonic reading and obstacle stopping
      4. (Optional) Data printing and timer for demonstration
    """

    # Buses for data exchange
    camera_bus = rr.Bus(None, "Camera Bus")
    interpretter_bus = rr.Bus(None, "Interpretation Bus")
    ultrasonic_bus = rr.Bus(None, "Ultrasonic Bus")
    bus_terminate = rr.Bus(0, "Termination Bus")

    # Camera tasks
    camera_writer = rr.Producer(
        sensor_cam,   # function generating camera frames
        camera_bus,      # bus to store camera frames
        0.05,         # delay
        bus_terminate,  # termination bus
        "Camera Capture"
    )

    interp_reader_writer = rr.ConsumerProducer(
        interpretter_cam,    # function processing frames to produce x_ratio
        camera_bus,       # input bus for frames
        interpretter_bus,    # output bus for x_ratio
        0.05,
        bus_terminate,
        "Line Interpretation"
    )

    angle_controller_consumer = rr.Consumer(
        angle_controller,  # function processing x_ratio
        interpretter_bus,        # input bus
        0.05,
        bus_terminate,
        "Steering Control"
    )

    # Ultrasonic tasks
    sonic_writer = rr.Producer(
        sonic_sensor_reading,
        ultrasonic_bus,
        0.05,
        bus_terminate,
        "Ultrasonic Reader"
    )

    sonic_reader = rr.Consumer(
        sonic_stop,
        ultrasonic_bus,
        0.05,
        bus_terminate,
        "Obstacle Stop"
    )

    # (Optional) Printer for debugging
    printer = rr.Printer(
        (camera_bus, interpretter_bus, ultrasonic_bus, bus_terminate),
        0.25,
        bus_terminate,
        "Data Printer",
        "Data bus readings: "
    )

    # (Optional) Termination timer (run for 5 seconds)
    termination_timer = rr.Timer(
        bus_terminate,
        5,
        0.01,
        bus_terminate,
        "Termination Timer"
    )

    # List of all producer-consumers to run
    producer_consumer_list = [
        camera_writer,
        interp_reader_writer,
        angle_controller_consumer,
        sonic_writer,
        sonic_reader,
        # Uncomment if you want to use:
        # printer,
        # termination_timer
    ]

    rr.runConcurrently(producer_consumer_list)


if __name__ == "__main__":
    main()
