import os
import logging
import math
# from logdecorator import log_on_start, log_on_end, log_on_error
try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic, utils
    on_the_robot = True
except ImportError:
    import sys
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic, utils
    on_the_robot = False
import time
import atexit

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)
                             
def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: trig, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P13', 'P12'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # reset robot_hat
        utils.reset_mcu()
        time.sleep(0.2)

        # --------- config_flie ---------
        if on_the_robot:
            self.config_flie = fileDB(config, 777, os.getlogin())
        else:
            self.config_flie = fileDB(config, 777, None)

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        trig, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(trig), Pin(echo, mode=Pin.IN, pull=Pin.PULL_DOWN))
        
    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # print(f"direction: {direction}, speed: {speed}")
        if speed != 0:
            speed = speed  # int(speed /2 ) + 50  #-- commented for 2.7(2)
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def backward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            # power_scale = (100 - abs_current_angle) / 100.0 
            if (current_angle / abs_current_angle) > 0:
                left_motor_speed, right_motor_speed = self.ackermann_wheel_speed(math.radians(abs_current_angle), speed, 0.5)
                logging.debug("[Backing Right: L>R] Changing motor speed to L: %.2f, R: %.2f"%(left_motor_speed, right_motor_speed))
                self.set_motor_speed(1, -left_motor_speed)
                self.set_motor_speed(2, right_motor_speed) 
            else:
                right_motor_speed, left_motor_speed = self.ackermann_wheel_speed(math.radians(abs_current_angle), speed, 0.5)
                logging.debug("[Backing Right: L<R] Changing motor speed to L: %.2f, R: %.2f"%(left_motor_speed, right_motor_speed))
                self.set_motor_speed(1, -left_motor_speed)
                self.set_motor_speed(2, right_motor_speed)
        else:
            logging.debug("Moving straight backward L: %.2f, R: %.2f"%(speed, speed))
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    def forward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            # power_scale = (100 - abs_current_angle) / 100.0
            if (current_angle / abs_current_angle) > 0:
                left_motor_speed, right_motor_speed = self.ackermann_wheel_speed(math.radians(abs_current_angle), speed, 0.5)
                logging.debug("[Turning Right: L>R] Changing motor speed to L: %.2f, R: %.2f"%(left_motor_speed, right_motor_speed))
                self.set_motor_speed(1,   left_motor_speed)
                self.set_motor_speed(2, -right_motor_speed) 
            else:
                right_motor_speed, left_motor_speed = self.ackermann_wheel_speed(math.radians(abs_current_angle), speed, 0.5)
                logging.debug("[Turning Right: L<R] Changing motor speed to L: %.2f, R: %.2f"%(left_motor_speed, right_motor_speed))
                self.set_motor_speed(1,   left_motor_speed)
                self.set_motor_speed(2, -right_motor_speed)
        else:
            logging.debug("Moving straight forward L: %.2f, R: %.2f"%(speed, speed))
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)       

    # function implemented for 2.7(3)
    def ackermann_wheel_speed(self, steering_angle, base_speed, gain):
        """
        Computes left and right wheel speeds using a sinusoidal approximation 
        to emulate Ackermann steering.

        :param steering_angle: float, steering angle in radians (or degrees if sin() is adjusted)
        :param base_speed: float, nominal translational speed
        :param gain: float, scaling factor k that adjusts turning sharpness
        :return: (float, float), left_wheel_speed, right_wheel_speed
        """
        # For simplicity, assume steering_angle is in radians
        left_wheel_speed = base_speed * (1.0 - gain * math.sin(steering_angle))
        right_wheel_speed = base_speed * (1.0 + gain * math.sin(steering_angle))
        return left_wheel_speed, right_wheel_speed

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def reset(self):
        self.stop()
        self.set_dir_servo_angle(0)
        self.set_cam_tilt_angle(0)
        self.set_cam_pan_angle(0)
    
    def move_discrete(self, direction: str, speed: float, angle: float = 0.0, duration: float = 2.0):
        """
        Move the car in a discrete manner.

        :param direction: 'forward' or 'backward'
        :param speed: speed value (positive for forward motion)
        :param angle: steering angle in degrees (default=0 for straight)
        :param duration: time in seconds to move before stopping
        """
        logging.debug("[move_discrete] direction: %s, speed: %.2f, angle: %.2f, duration: %.2f",
                    direction, speed, angle, duration)

        # Constrain the steering angle, set it, then move
        self.set_dir_servo_angle(angle)
        
        if direction.lower() == 'forward':
            self.forward(speed)
        elif direction.lower() == 'backward':
            self.backward(speed)
        else:
            logging.warning("Invalid direction specified. Use 'forward' or 'backward'.")
            return

        time.sleep(duration)
        self.stop()

    def parallel_park_left(self, speed=30):
        """
        Perform a simplified parallel parking maneuver to the left.

        Steps:
        1) Turn wheels fully left and reverse for short duration.
        2) Turn wheels fully right and continue reversing to straighten out.
        3) Move forward slightly to finalize position.
        """
        logging.info("[parallel_park_left] Starting parallel parking to the left.")

        # Step 1: Turn wheels left (assume max angle is self.DIR_MAX) and back up
        self.set_dir_servo_angle(self.DIR_MAX)    # Max left
        self.backward(speed)
        time.sleep(1.5)   # Adjust time as necessary
        self.stop()

        # Step 2: Turn wheels right (negative angle) and reverse more
        self.set_dir_servo_angle(-self.DIR_MAX)   # Max right
        self.backward(speed)
        time.sleep(1.5)
        self.stop()

        # Step 3: Straighten out and move forward to finalize position
        self.set_dir_servo_angle(0)
        self.forward(speed)
        time.sleep(1.0)
        self.stop()
        logging.info("[parallel_park_left] Completed.")

    def parallel_park_right(self, speed=30):
        """
        Perform a simplified parallel parking maneuver to the right.
        """
        logging.info("[parallel_park_right] Starting parallel parking to the right.")

        # Step 1: Turn wheels right and back up
        self.set_dir_servo_angle(-self.DIR_MAX)  # Max right
        self.backward(speed)
        time.sleep(1.5)
        self.stop()

        # Step 2: Turn wheels left (positive angle) and reverse more
        self.set_dir_servo_angle(self.DIR_MAX)   # Max left
        self.backward(speed)
        time.sleep(1.5)
        self.stop()

        # Step 3: Straighten out and move forward
        self.set_dir_servo_angle(0)
        self.forward(speed)
        time.sleep(1.0)
        self.stop()
        logging.info("[parallel_park_right] Completed.")

    def three_point_turn_left(self, speed=30):
        """
        Perform a simple three-point (K) turn starting to the left.
        1) Forward left
        2) Backward right
        3) Forward left
        """
        logging.info("[three_point_turn_left] Starting three-point turn (left).")

        # 1) Forward left
        self.set_dir_servo_angle(self.DIR_MAX)  # Turn wheels left
        self.forward(speed)
        time.sleep(1.5)
        self.stop()

        # 2) Backward right
        self.set_dir_servo_angle(-self.DIR_MAX) # Turn wheels right
        self.backward(speed)
        time.sleep(1.5)
        self.stop()

        # 3) Forward left again
        self.set_dir_servo_angle(self.DIR_MAX)
        self.forward(speed)
        time.sleep(1.5)
        self.stop()

        # Straighten wheels
        self.set_dir_servo_angle(0)
        logging.info("[three_point_turn_left] Completed.")

    def three_point_turn_right(self, speed=30):
        """
        Perform a simple three-point (K) turn starting to the right.
        1) Forward right
        2) Backward left
        3) Forward right
        """
        logging.info("[three_point_turn_right] Starting three-point turn (right).")

        # 1) Forward right
        self.set_dir_servo_angle(-self.DIR_MAX)
        self.forward(speed)
        time.sleep(1.5)
        self.stop()

        # 2) Backward left
        self.set_dir_servo_angle(self.DIR_MAX)
        self.backward(speed)
        time.sleep(1.5)
        self.stop()

        # 3) Forward right
        self.set_dir_servo_angle(-self.DIR_MAX)
        self.forward(speed)
        time.sleep(1.5)
        self.stop()

        # Straighten wheels
        self.set_dir_servo_angle(0)
        logging.info("[three_point_turn_right] Completed.")


    
    atexit.register(stop)

if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    time.sleep(1)
    px.stop()
