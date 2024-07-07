import time
import signal
import sys
import RPi.GPIO as GPIO
from robot_movement import RobotMovement
from servo_control import ServoControl
from eit_voltage_check import EITVoltageCheck
from tof_control import TOFControl

class MainControl:
    def __init__(self):
        self.robot = RobotMovement()
        self.servo = ServoControl()
        self.eit = EITVoltageCheck()
        self.tof = TOFControl()

    def initialize_components(self):
        # Initialize all components and wait for stabilization
        self.servo.move_to_initial_positions()
        time.sleep(5)  # Allow components to stabilize

    def move_servos_upward(self):
        # Move servo motors to initial angles
        self.servo.move_servos_upward()
        time.sleep(1)  # Allow time for servo motors to move

    def move_forward_and_stop(self):
        # Move forward for 1 second and then stop for 20 seconds
        self.robot.move_forward()
        time.sleep(1)
        self.robot.stop_motors()
        time.sleep(20)

    def perform_voltage_check(self):
        # Perform an EIT voltage check
        self.eit.run()

    def move_servos_downward(self):
        # Immediately move servo motors downward
        self.servo.move_servos_downward()
        time.sleep(15)  # Wait for 15 seconds

    def handle_distance_detection(self):
        # Handle distance detection using ToF sensors
        while True:
            front_distance = self.tof.get_distance('sensor_front')
            side_distance_left = self.tof.get_distance('sensor_left')
            side_distance_right = self.tof.get_distance('sensor_right')

            if front_distance <= 100 and side_distance_left <= 50 and side_distance_right <= 50:
                # Detected conditions met, stop motors
                self.robot.stop_motors()
                break

            # Interrupt mechanism
            if front_distance <= 100:
                # On detection of close distance by front ToF sensor
                self.handle_left_turn()

    def handle_left_turn(self):
        # Take a left turn and continue forward
        self.robot.turn_left()
        time.sleep(0.2)
        self.robot.stop_motors()
        self.robot.turn_left()
        self.robot.move_forward()

    def run(self):
        try:
            self.initialize_components()

            while True:
                self.servo.move_servos_upward()
                self.move_forward_and_stop()
                self.move_servos_downward()
                self.perform_voltage_check()
                self.servo.move_servos_upward()

                self.handle_distance_detection()

                # Ensure cleanup on exit
                signal.signal(signal.SIGINT, self.signal_handler)
                signal.pause()

        except KeyboardInterrupt:
            print("\nProgram interrupted. Cleaning up...")
            self.cleanup()

    def signal_handler(self, signal, frame):
        print("Signal received. Cleaning up...")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        self.robot.cleanup()
        self.servo.cleanup()
        self.eit.cleanup()
        GPIO.cleanup()
        sys.exit(0)

if __name__ == "__main__":
    main_control = MainControl()
    main_control.run()
