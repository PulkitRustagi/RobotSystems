import time
import logging
from picarx_improved import Picarx
def main():
    """
    Continuously prompts the user for numeric keyboard commands and executes the
    corresponding maneuver on the Picarx robot until the user enters '0' to quit.
    """
    # Instantiate your Picarx object
    px = Picarx()
    logging.info("Initialized Picarx. Enter numeric commands to maneuver or '0' to quit.")

    # Dictionary mapping numeric strings to lambda functions that call Picarx methods
    # Adjust angles, speeds, and durations as needed for your platform.
    command_map = {
        '1': lambda: px.move_discrete('forward',  30, angle=0,          duration=1.5),
        '2': lambda: px.move_discrete('backward', 30, angle=0,          duration=1.5),
        '3': lambda: px.move_discrete('forward',  30, angle= px.DIR_MAX,  duration=1.5),
        '4': lambda: px.move_discrete('forward',  30, angle=-px.DIR_MAX, duration=1.5),
        '5': lambda: px.move_discrete('backward', 30, angle= px.DIR_MAX,  duration=1.5),
        '6': lambda: px.move_discrete('backward', 30, angle=-px.DIR_MAX, duration=1.5),
        '7': lambda: px.parallel_park_left(speed=30),
        '8': lambda: px.parallel_park_right(speed=30),
        '9': lambda: px.three_point_turn_left(speed=30),
        '10': lambda: px.three_point_turn_right(speed=30)
        # Add additional custom commands if desired
    }

    while True:
        # Prompt user for numeric input
        user_input = input(
            "\nEnter a numeric command:\n"
            "  1) Forward\n"
            "  2) Backward\n"
            "  3) Forward (turn left)\n"
            "  4) Forward (turn right)\n"
            "  5) Backward (turn left)\n"
            "  6) Backward (turn right)\n"
            "  7) Parallel park left\n"
            "  8) Parallel park right\n"
            "  9) Three-point turn left\n"
            "  10) Three-point turn right\n"
            "  0) Quit\n"
            "Your choice: "
        ).strip()

        # Check for quit command
        if user_input == '0':
            logging.info("Quitting program. Stopping the robot.")
            px.stop()  # Make sure the robot is stopped
            break

        # Execute the corresponding maneuver if the command exists
        if user_input in command_map:
            try:
                command_map[user_input]()
            except Exception as e:
                logging.warning(f"An error occurred while executing '{user_input}': {e}")
        else:
            logging.warning(f"Unrecognized command: '{user_input}'. Please try again.")

        # Small delay to avoid saturating the loop
        time.sleep(0.1)

if __name__ == "__main__":
    main()
