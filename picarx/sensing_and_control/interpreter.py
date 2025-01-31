import numpy as np

class Interpreter():
    def __init__(self, line_threshold=35, sensitivity=1.0, is_dark_line=True):
        # class variables
        self.sensitivity = sensitivity
        self.is_dark_line = is_dark_line
        self.line_threshold = line_threshold
        self.sensor_with_line_last_detected = 1
        self.prev_turn_proportion = 0
        # storage for PID controller
        self.last_error = 0
        self.sum_error = 0

    def interpret_sensor_reading_discrete(self, sensor_reading, threshold=20):
        if(self.has_no_significant_difference(sensor_reading)):
            if(self.sensor_with_line_last_detected == 1):
                return(self.prev_turn_proportion)
            else:
                self.prev_turn_proportion = 1-self.sensor_with_line_last_detected
                return(1-self.sensor_with_line_last_detected)


    def get_turn_proportion(self, avg_difference, scaling_function, threshold):
        # Change the threshold according to the sensitivity
        threshold *= self.sensitivity

        # scaling functions ranked from least to most sensitive around near-zero angles
        if(scaling_function == "cubic"):
            # threshold is where the value is 1
            x = avg_difference/threshold
            turn_proportion = np.power(x, 3)
            # out of bounds case
            if(turn_proportion < -1):
                turn_proportion = -1.0
            elif(turn_proportion > 1):
                turn_proportion = 1.0
            # return turn proportion
            return(turn_proportion)


        elif(scaling_function == "square"):
            # threshold is where the value is 1
            x = avg_difference/threshold
            turn_proportion = np.power(x, 2) if avg_difference > 0 else -np.power(x, 2)
            # out of bounds case
            if(turn_proportion < -1):
                turn_proportion = -1.0
            elif(turn_proportion > 1):
                turn_proportion = 1.0
            # return turn proportion
            return(turn_proportion)


        elif(scaling_function == "linear"):
            # threshold is where the value is 1
            x = avg_difference/threshold
            turn_proportion = x
            # out of bounds case
            if(turn_proportion < -1):
                turn_proportion = -1.0
            elif(turn_proportion > 1):
                turn_proportion = 1.0
            # return turn proportion
            return(turn_proportion)


        elif(scaling_function == "sin"):
            # threshold is the place where the proportion output is at +/-1
            x = avg_difference*(1.571/threshold)
            turn_proportion = np.sin(x)
            # out of bounds case
            if(avg_difference < -threshold):
                turn_proportion = -1.0
            elif(avg_difference > threshold):
                turn_proportion = 1.0
            # return turn proportion
            return(turn_proportion)


        elif(scaling_function == "logistic"):
            # threshold is the place where logistic is at 99% turn angle
            x = avg_difference*(5.293/threshold)
            turn_proportion = 2/(1 + np.exp(-x)) - 1
            # return turn proportion
            return(turn_proportion)


        else:
            return(0)

            
    def interpret_sensor_reading_proportional(self, sensor_reading, scaling_function="linear", threshold=25):
        if(self.has_no_significant_difference(sensor_reading)):
            if(self.sensor_with_line_last_detected == 1):
                return(self.prev_turn_proportion)
            else:
                self.prev_turn_proportion = 1-self.sensor_with_line_last_detected
                return(1-self.sensor_with_line_last_detected)
        
        left_avg = np.mean(sensor_reading[0:2])
        right_avg = np.mean(sensor_reading[1:3])

        avg_difference = left_avg - right_avg

        turn_proportion = self.get_turn_proportion(avg_difference, scaling_function=scaling_function, threshold=threshold)
        print('(', left_avg, right_avg, ')', avg_difference, turn_proportion)
        
        self.prev_turn_proportion = turn_proportion
        return(turn_proportion)

    
    def interpret_sensor_reading_PID(self, sensor_reading, k_p=1.0, k_i=0.0, k_d=0.0):
        if(self.has_no_significant_difference(sensor_reading)):
            if(self.sensor_with_line_last_detected == 1):
                return(self.prev_turn_proportion)
            else:
                self.prev_turn_proportion = 1-self.sensor_with_line_last_detected
                return(1-self.sensor_with_line_last_detected)
        # referencing: https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=9013
        left_avg = np.mean(sensor_reading[0:2])
        right_avg = np.mean(sensor_reading[1:3])

        error = left_avg - right_avg

        pid = k_p*(error) + k_i*(self.sum_error + error) + k_d*(error - self.last_error)

        pid *= 0.05

        turn_proportion = 2/(1 + np.exp(-pid)) - 1
        print(left_avg, '-', right_avg, '=', error)
        print(pid, '=', turn_proportion)

        self.sum_error += error
        self.last_error = error
        
        self.prev_turn_proportion = turn_proportion
        return(turn_proportion)


    def has_no_significant_difference(self, sensor_reading):
        # if working with light line, then flip sensor readings such that the max value is the line
        if(not self.is_dark_line):
            sensor_reading *= -1
        # get the reading that is suspect to be the line
        line_index = np.argmax(sensor_reading)
        floor_index = [0, 1, 2]
        floor_index.remove(line_index)
        # check that the difference of the line reading to the average surrounding reading is at least more than the threshold
        line_reading = sensor_reading[line_index]
        avg_floor_reading = np.mean(sensor_reading[floor_index])
        floor_line_difference = np.abs(line_reading - avg_floor_reading)
        #return whether the difference is significant
        if(floor_line_difference <= (self.line_threshold/self.sensitivity)):
            return(True)
        else:
            print(floor_line_difference)
            self.sensor_with_line_last_detected = line_index
            return(False)