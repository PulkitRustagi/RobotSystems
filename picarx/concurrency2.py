import cv2
import numpy as np
from vilib import Vilib
from picarx_improved import Picarx
from sensing_and_control.CONTROL import Controller as CONTROL
from time import sleep
from concurrent.futures import ThreadPoolExecutor
from threading import Event
from readerwriterlock import rwlock
import time

px = Picarx()

# Define shutdown event
shutdown_event = Event()

class Bus:
    def __init__(self):
        self.message = None
        self.lock = rwlock.RWLockWriteD()
        self.identifier = None

    def read(self):
        with self.lock.gen_rlock():
            return(self.message, self.identifier)

    def write(self, data, identifier = None):
       with self.lock.gen_wlock():
            self.message = data
            self.identifier = identifier

bus12 = Bus()  #Bus instance for transfer from sensor to interpreter

bus23 = Bus() #Bus instance for transfer from interpreter to controller

sensor_delay = 0.02
interp_delay = 0.1
control_delay = 0.2
#Producer function
def sensor(bus12, sensor_delay):

    Vilib.camera_start()
    Vilib.display()
    time.sleep(0.2)
    t = 1
    while not shutdown_event.is_set():
        name = f"image{t}"  
        path = "picarx"

        status = Vilib.take_photo(name, path)
        if status:
            full_path = f"{path}/{name}.jpg"
            if Vilib.img is not None and isinstance(Vilib.img, np.ndarray):
                cv2.imwrite(full_path, Vilib.img)  # Save the image
                t += 1
                frame = cv2.imread(f'{path}/{name}.jpg')
                #print("Image", t, "saved successfully.")
                bus12.write(frame, name)
                print(f"Image {name} written to bus12")
                print
                
            else:
                print("Image not valid")
                continue

        time.sleep(sensor_delay)
    
#Read and write function

def interpreter(bus12, bus23,interp_delay):

    while not shutdown_event.is_set():
        frame, identifier = bus12.read()

        if frame is None:
            print("No frame available")
            time.sleep(interp_delay)  # Sleep briefly to avoid busy waiting
            continue

        print(f"{identifier} picked up from bus 12")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        #Get binary image
        _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        
        #Focus on bottom half of image
        frame_height = frame.shape[0]
        frame_width = frame.shape[1]

        lower_half = binary[frame_height//2:,:]
        #Find contours
        contours, _ = cv2.findContours(lower_half, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        center_line = max(contours, key = cv2.contourArea)

        
        # Find centroid point of the largest contour
        centroid = cv2.moments(center_line)
        if centroid['m00'] !=0:
            x_center = int(centroid['m10'] / centroid['m00'])
            x_ratio = (x_center - frame_width / 2) / (frame_width / 2) #Get x in terms of -1 to 1
            # Draw a red dot at the x_center position
            # cv2.circle(frame, (x_center, frame.shape[0] // 2), 5, (0, 0, 255), -1)  # Red dot
            # # Save the modified image
            # cv2.imwrite(f'{path}/{name}.jpg', frame)

        if x_ratio is not None:
            bus23.write(x_ratio)
            print("x_ratio written to bus23", x_ratio)
        else:
            print("x was None")

        time.sleep(interp_delay)


#Consumer function -- Changes angle based on x_value
def controller(bus23,control_delay):
    C = CONTROL(px, scale_factor = 30)
    
    while not shutdown_event.is_set():
        x, identifier = bus23.read()
        if x is None:
            print("NO X VALUE FROM BUS 23")
            time.sleep(control_delay)
            continue
        print("X RATIO CONTROLLER YIPPIE:", x)
        xval = -1*x
        C.correct_car(xval)
        time.sleep(control_delay)
        px.forward(25)

# Exception handle function
def handle_exception(future):
    exception = future.exception()
    if exception:
        print(f'Exception in worker thread: {exception}')

# Define robot task
def robot_task(i):
    print('Starting robot task', i)
    while not shutdown_event.is_set():
        # Run some robot task...
        print('Running robot task', i)
        sleep(1)
    # Print shut down message
    print('Shutting down robot task', i)
    # Test exception
    if i == 1:
        raise Exception('Robot task 1 raised an exception')


start = input("Press 'y' to begin line following. Once started, press 's' to stop")

px.set_cam_tilt_angle(-30)

with ThreadPoolExecutor(max_workers=3) as executor:
    eSensor = executor.submit(sensor, bus12,
    sensor_delay)
    eInterpreter = executor.submit(interpreter,
    bus12, bus23,interp_delay)
    eController = executor.submit(controller,bus23,control_delay)

    # Add exception call back
    eSensor.add_done_callback(handle_exception)
    eInterpreter.add_done_callback(handle_exception)
    eController.add_done_callback(handle_exception)

    try:
        # Keep the main thread running to respond to the kill signal
        while not shutdown_event.is_set():
            sleep(0.2)
    except KeyboardInterrupt:
        # Trigger the shutdown event when receiving the kill signal
        print('Shutting down')
        shutdown_event.set()
    finally:
        # Ensures all threads finish
        executor.shutdown()



