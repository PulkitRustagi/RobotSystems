#!/usr/bin/env python3
"""
Example: Multimodal Control with RossROS
----------------------------------------
Demonstrates how to use RossROS to handle two sensor streams:
1) A line sensor (camera or photocells) for line following
2) An ultrasonic sensor to detect obstacles

We run concurrent Producer, ConsumerProducer, and Consumer threads that share data via Busses,
and a Timer that terminates everything after a set duration.

Adapt sensor reads, control logic, and thresholds to your specific hardware.
"""

import time
import logging

# If RossROS.py is in the same directory, this import should work:
from RossROS import Bus, Producer, ConsumerProducer, Consumer, Timer

# (Optional) If you're using PiCar-X, you might import:
# from picarx import Picarx

###############################################################################
#                             Sensor / Interpreter / Controller Functions
###############################################################################

def line_sensor_function():
    """
    Reads line sensor data (e.g., camera or photocells).
    Return a dict representing raw sensor values or partial processing.
    For demonstration, we return a placeholder.
    """
    # e.g.:
    # line_value = read_line_sensors() or compute_camera_line_offset()
    # return {"line_raw": line_value}
    return {"line_raw": 123}  # Placeholder


def line_interpreter_function(input_data):
    """
    Interprets line sensor data to produce an offset or steering guide.
    input_data is the dict from line_sensor_function (e.g., {"line_raw": 123}).
    Return a dict of interpreted values, e.g., {"line_offset": 0.25}.
    """
    # Example placeholder logic:
    raw_val = input_data["line_raw"]
    # Suppose we pretend the offset is some function of raw_val
    offset = (raw_val - 120) / 100.0  # Arbitrary placeholder
    return {"line_offset": offset}


def ultrasonic_sensor_function():
    """
    Reads the ultrasonic distance (in cm, for example) and returns a dict.
    E.g., on PiCar-X: distance = px.get_distance()
    """
    # distance = px.get_distance()  # Actual call if using PiCar-X
    distance = 50  # Placeholder
    return {"distance": distance}


def ultrasonic_interpreter_function(input_data):
    """
    Interprets ultrasonic data and returns whether an obstacle is detected.
    input_data = {"distance": <float>}
    """
    distance = input_data["distance"]
    threshold = 20.0  # cm; obstacle if distance < threshold
    obstacle_detected = (distance is not None) and (distance < threshold)
    return {"obstacle_detected": obstacle_detected}


def combined_controller_function(input_list):
    """
    Reads from 2 inputs:
       1) line_data (e.g., {"line_offset": 0.25})
       2) ultrasonic_data (e.g., {"obstacle_detected": True/False})
    Decides motor/steering commands.

    You can integrate a PiCar-X control by creating a Picarx() instance 
    outside in the main thread and referencing it globally, or using 
    a function closure approach. For demonstration, we just log actions.
    """
    line_data, us_data = input_list

    # If we haven't gotten valid data yet, do nothing
    if (line_data is None) or (us_data is None):
        logging.info("No data yet. Controller idle.")
        return

    line_offset = line_data.get("line_offset", 0.0)
    obstacle = us_data.get("obstacle_detected", False)

    if obstacle:
        # Example: stop the motors if an obstacle is detected
        # px.stop()
        logging.info(f"Obstacle detected! Stopping. Offset was {line_offset:.2f}")
    else:
        # Example: line-following
        # px.set_dir_servo_angle( line_offset * GAIN )
        # px.forward(speed)
        logging.info(f"No obstacle. Following line. Offset={line_offset:.2f}")


###############################################################################
#                                     MAIN
###############################################################################

def main():
    logging.basicConfig(level=logging.INFO)
    logging.info("Starting Multimodal Control Demo with RossROS...")

    # 1. Create your bus objects
    line_sensor_bus = Bus()               # raw line data
    line_interpreter_bus = Bus()          # interpreted line offset
    ultrasonic_bus = Bus()                # raw distance data
    ultrasonic_interpreter_bus = Bus()    # obstacle status
    termination_bus = Bus(False)          # read by all to know when to stop

    # 2. Define your Producers / ConsumerProducers / Consumers / Timer

    # --- Producer for line sensor ---
    line_sensor_producer = Producer(
        producer_function=line_sensor_function,
        output_bus=line_sensor_bus,
        termination_busses=[termination_bus],
        delay=0.2  # read line sensor ~5 times/sec
    )

    # --- ConsumerProducer for line interpretation ---
    line_interpreter_consumerproducer = ConsumerProducer(
        consumer_function=line_interpreter_function,
        input_busses=[line_sensor_bus],
        output_bus=line_interpreter_bus,
        termination_busses=[termination_bus],
        delay=0.2
    )

    # --- Producer for ultrasonic sensor ---
    ultrasonic_producer = Producer(
        producer_function=ultrasonic_sensor_function,
        output_bus=ultrasonic_bus,
        termination_busses=[termination_bus],
        delay=0.2  # read ultrasonic sensor ~5 times/sec
    )

    # --- ConsumerProducer for ultrasonic interpretation ---
    ultrasonic_interpreter_consumerproducer = ConsumerProducer(
        consumer_function=ultrasonic_interpreter_function,
        input_busses=[ultrasonic_bus],
        output_bus=ultrasonic_interpreter_bus,
        termination_busses=[termination_bus],
        delay=0.2
    )

    # --- Consumer for combined control ---
    # Reads line_interpreter_bus and ultrasonic_interpreter_bus
    combined_controller_consumer = Consumer(
        consumer_function=combined_controller_function,
        input_busses=[line_interpreter_bus, ultrasonic_interpreter_bus],
        termination_busses=[termination_bus],
        delay=0.1
    )

    # --- Timer (Producer) for automatic termination after 30s ---
    run_time = 30.0
    timer_producer = Timer(
        t_duration=run_time,
        output_bus=termination_bus
    )

    # 3. Put all processes in a list
    process_list = [
        line_sensor_producer,
        line_interpreter_consumerproducer,
        ultrasonic_producer,
        ultrasonic_interpreter_consumerproducer,
        combined_controller_consumer,
        timer_producer
    ]

    # 4. Start them all
    for process in process_list:
        process.start()

    # 5. Wait for them to finish (i.e., until termination_bus becomes True)
    for process in process_list:
        process.join()

    logging.info("All processes have shut down. Exiting program.")


if __name__ == "__main__":
    main()
