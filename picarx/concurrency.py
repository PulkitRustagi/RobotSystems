import cv2
import numpy as np
from picarx_improved import Picarx
from sensing_and_control.CONTROL import Controller as CONTROLLER
from time import sleep
from concurrent.futures import ThreadPoolExecutor
from threading import Event
from readerwriterlock import rwlock
from vilib import Vilib
from time import sleep, time


# Define shutdown event
shutdown_event = Event()


def take_photo(t):
    Vilib.camera_start(vflip=False,hflip=False)
    name = f'cam_image{t}'
    path = f'picarx/'
    Vilib.take_photo(name, path)
    img = cv2.imread(f'{path}{name}.jpg')
    if img is None:
        print(f"---------------\nNo image available at {path}{name}.jpg")
        exit(0)
    return img


class Bus:
    def __init__(self):
        self.message = None
        self.lock = rwlock.RWLockWriteD()
        self.identifier = None

    def read(self):
        with self.lock.gen_rlock():
            return(self.message, self.identifier)

    def write(self, data, identifier = None):
       print("Trying Writing to bus so I code is at point 1")
       with self.lock.gen_wlock():
            print("Also got into the with block")
            self.message = data
            self.identifier = identifier
            print(f"Data written to bus: {identifier}")



def sensor(bus1, sensor_delay):
    t = 1
    name = f'cam_image{t}'
    path = f'picarx/'
    time.sleep(0.2)
    print("Sensor started")
    print("Shutting event is set: ", shutdown_event.is_set())
    while not shutdown_event.is_set():
        print("Trying to take photo\n\n")
        img = take_photo(t)
        t += 1
        print("Status of taking photo: ", img!=None)
        if img is not None:
            if isinstance(Vilib.img, np.ndarray):
                bus1.write(img, name)
                print(f"Image {name} sent via bus1\n")
            else:
                print("No image available")
                continue

        time.sleep(sensor_delay)

def interpreter(bus1, bus2, interp_delay):

    while not shutdown_event.is_set():
        frame, identifier = bus1.read()

        if frame is None:
            print("No frame available")
            time.sleep(interp_delay)
            continue

        print(f"{identifier} picked up from bus1")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        
        # Focus on bottom half of image
        frame_height, frame_width = frame.shape[0:2]
        lower_half = binary[frame_height//2:,:]

        # Find contours
        contours, _ = cv2.findContours(lower_half, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center_line = max(contours, key = cv2.contourArea)
        
        # Find centroid point of the largest contour
        centroid = cv2.moments(center_line)
        if centroid['m00'] != 0:
            x_center = int(centroid['m10'] / centroid['m00'])
            x_pos = (x_center - frame_width / 2) / (frame_width / 2) # [-1, 1]

        if x_pos is not None:
            bus2.write(x_pos)
            print("Line position [-1, 1] sent via bus2: ", x_pos)
        else:
            print("Error in calculating line position")
        time.sleep(interp_delay)

def controller(px, bus2, control_delay):
    C = CONTROLLER(px, scale_factor = 30)
    while not shutdown_event.is_set():
        x, identifier = bus2.read()
        if x is None:
            time.sleep(control_delay)
            continue
        xval = -x
        C.correct_car(xval)
        time.sleep(control_delay)
        px.forward(30)

# Exception handle function
def handle_exception(future):
    exception = future.exception()
    if exception:
        print(exception)

# Define robot task
def robot_task(i):
    print('Starting Task ', i)
    while not shutdown_event.is_set():
        print('Running Task ', i)
        sleep(1)
    print('Finishing Task ', i)
    if i == 1:
        raise Exception('Robot task 1 raised an exception')

def main():

    px = Picarx()
    bus1 = Bus() # Bus sensor to interpreter
    bus2 = Bus() # Bus interpreter to controller

    # Set delay for loops based on polling speeds
    # Sense > Interpret > Contol
    sensor_delay  = 0.02
    interp_delay  = 0.10
    control_delay = 0.20

    px.set_cam_tilt_angle(-30)

    with ThreadPoolExecutor(max_workers=3) as executor:
        eSensor = executor.submit(sensor, bus1, sensor_delay)
        eInterpreter = executor.submit(interpreter, bus1, bus2, interp_delay)
        eController = executor.submit(controller, bus2, control_delay)

        # Add exception call back
        eSensor.add_done_callback(handle_exception)
        eInterpreter.add_done_callback(handle_exception)
        eController.add_done_callback(handle_exception)

        try:
            while not shutdown_event.is_set():
                sleep(0.2)
        except KeyboardInterrupt:
            print('Shutting down')
            shutdown_event.set()
        finally:
            executor.shutdown()



if __name__ == "__main__":
    main()