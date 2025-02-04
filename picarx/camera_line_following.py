import time
import logging

from picarx_improved import Picarx
from vilib import Vilib

def main():
    """
    Demonstrates camera-based line following on PiCar-X with the camera tilted downward.
    Assumes a black line on a lighter floor.
    """

    # 1. Initialize PiCar-X
    px = Picarx()
    px.stop()  # Ensure it's not moving at the start

    # 2. Tilt the camera downward so it can see the floor.
    #    Adjust the angle as needed for your physical setup.
    #    Positive angles typically tilt up, negative angles tilt down for PiCar-X v2.0 hardware.
    CAMERA_TILT_ANGLE = -20
    px.set_cam_tilt_angle(CAMERA_TILT_ANGLE)
    logging.info(f"Camera tilted to {CAMERA_TILT_ANGLE} degrees.")

    # 3. Start the Vilib camera
    Vilib.camera_start(vflip=False, hflip=False)
    # Use a smaller resolution for faster processing
    Vilib.config_size('320x240')
    # If you want local or web display for debugging (pick one):
    # Vilib.display(local=True, web=False)   # Local OpenCV window (requires desktop/VNC)
    # Vilib.display(local=False, web=True)   # Web streaming (check console for the URL & port)

    # 4. Enable line tracking and set line color to black
    Vilib.line_tracking_switch(True)
    Vilib.set_line_color('black')

    # 5. Driving parameters
    speed = 15         # Base driving speed [0..100]
    angle_gain = 20.0  # Proportional factor for steering from offset
    loop_delay = 0.05  # Seconds between loop iterations

    try:
        while True:
            # a) Retrieve the line offset from the camera center
            offset = Vilib.line_pos_x  # None if no line is detected

            if offset is None:
                # If no line is found, optionally stop or keep going straight
                px.stop()
                logging.info("No line detected; stopping.")
            else:
                # b) Normalize offset (for 320 width, half-width = 160)
                normalized_offset = offset / 160.0  # ~[-1..1]
                # c) Convert offset to servo angle
                steering_angle = angle_gain * normalized_offset

                # d) Command steering
                px.set_dir_servo_angle(steering_angle)

                # e) Move forward
                px.forward(speed)

                # Debug info
                logging.info(f"Offset: {offset:.1f}, Steering: {steering_angle:.1f}")

            time.sleep(loop_delay)

    except KeyboardInterrupt:
        logging.info("User interrupted. Exiting...")
    finally:
        # Cleanup
        px.stop()
        Vilib.line_tracking_switch(False)
        Vilib.camera_close()
        # If you had a local or web display open, you can disable it here:
        # Vilib.display(local=False, web=False)
        logging.info("Program terminated cleanly.")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
