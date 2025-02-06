import time
import logging
import concurrent.futures
from threading import Event

# Import the bus class and the consumer-producer functions
# from your_module import Bus, sensor_function, interpreter_function, controller_function

# For illustration, we define them inline here:
from readerwriterlock import rwlock

class Bus:
    def __init__(self, initial_value=None):
        self.message = initial_value

    def write(self, new_message):
        self.message = new_message

    def read(self):
        return self.message

def sensor_function(sensor_bus, delay, shutdown_event):
    while not shutdown_event.is_set():
        sensor_data = {"some_sensor_value": 123}  # placeholder
        sensor_bus.write(sensor_data)
        time.sleep(delay)
    print("[sensor_function] Exiting loop.")

def interpreter_function(sensor_bus, interpreter_bus, delay, shutdown_event):
    while not shutdown_event.is_set():
        data = sensor_bus.read()
        if data:
            # interpret data
            interpretation = {"offset": 0.42}  # placeholder
            interpreter_bus.write(interpretation)
        time.sleep(delay)
    print("[interpreter_function] Exiting loop.")

def controller_function(interpreter_bus, delay, shutdown_event):
    while not shutdown_event.is_set():
        interpretation = interpreter_bus.read()
        if interpretation:
            # use interpretation for control
            pass
        time.sleep(delay)
    print("[controller_function] Exiting loop.")

def handle_exception(future):
    """Callback function to handle exceptions in worker threads."""
    exc = future.exception()
    if exc:
        logging.error(f"Exception in worker thread: {exc}")

def main():
    logging.basicConfig(level=logging.INFO)
    print("Starting concurrency demo...")

    # 1. Create busses
    sensor_bus = Bus()
    interpreter_bus = Bus()

    # 2. Create a shutdown event
    shutdown_event = Event()

    # 3. Number of workers needed: at least 3 for sensor, interpreter, controller
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:

        # 4. Submit tasks to the executor
        future_sensor = executor.submit(sensor_function, sensor_bus, 0.5, shutdown_event)
        future_sensor.add_done_callback(handle_exception)

        future_interpreter = executor.submit(interpreter_function, sensor_bus, interpreter_bus, 0.5, shutdown_event)
        future_interpreter.add_done_callback(handle_exception)

        future_controller = executor.submit(controller_function, interpreter_bus, 0.5, shutdown_event)
        future_controller.add_done_callback(handle_exception)

        try:
            # 5. Keep main thread alive until Ctrl-C
            while not shutdown_event.is_set():
                time.sleep(1)
        except KeyboardInterrupt:
            logging.info("Caught KeyboardInterrupt, shutting down...")
            shutdown_event.set()

        # 6. Once we exit the 'with' block, executor.shutdown() is implicitly called
    print("All threads have exited. Program complete.")

if __name__ == "__main__":
    main()
