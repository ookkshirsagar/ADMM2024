import RPi.GPIO as GPIO
import time

class ServoControl:
    def __init__(self):
        # Set GPIO numbering mode
        GPIO.setmode(GPIO.BCM)

        # Set GPIO pins for the servos
        self.servo_pin_1 = 17  # GPIO pin for the first servo
        self.servo_pin_2 = 27  # GPIO pin for the second servo
        self.servo_pin_3 = 22  # GPIO pin for the third servo
        self.servo_pin_4 = 4   # GPIO pin for the fourth servo

        # Set PWM parameters
        GPIO.setup(self.servo_pin_1, GPIO.OUT)
        GPIO.setup(self.servo_pin_2, GPIO.OUT)
        GPIO.setup(self.servo_pin_3, GPIO.OUT)
        GPIO.setup(self.servo_pin_4, GPIO.OUT)

        # PWM frequency set to 50 Hz for all servos
        self.pwm_1 = GPIO.PWM(self.servo_pin_1, 50)
        self.pwm_2 = GPIO.PWM(self.servo_pin_2, 50)
        self.pwm_3 = GPIO.PWM(self.servo_pin_3, 50)
        self.pwm_4 = GPIO.PWM(self.servo_pin_4, 50)

        # Start PWM with 0% duty cycle
        self.pwm_1.start(0)
        self.pwm_2.start(0)
        self.pwm_3.start(0)
        self.pwm_4.start(0)

        # Initial angles for the servos
        self.initial_angle_1 = 170
        self.initial_angle_2 = 0
        self.initial_angle_3 = 170
        self.initial_angle_4 = 0

        # Set initial positions for the servos
        self.set_servo_angle(self.pwm_1, self.initial_angle_1)
        self.set_servo_angle(self.pwm_2, self.initial_angle_2)
        self.set_servo_angle(self.pwm_3, self.initial_angle_3)
        self.set_servo_angle(self.pwm_4, self.initial_angle_4)
        time.sleep(1)  # Give time for servos to reach the initial positions

    def set_servo_angle(self, pwm, angle):
        duty_cycle = 2.5 + (angle / 18.0)  # Adjust the duty cycle calculation if needed
        pwm.ChangeDutyCycle(duty_cycle)

    def move_servos(self):
        try:
            while True:
                # Move the servos to specific angles
                self.set_servo_angle(self.pwm_1, 140)
                self.set_servo_angle(self.pwm_2, 30)
                self.set_servo_angle(self.pwm_3, 140)
                self.set_servo_angle(self.pwm_4, 30)
                time.sleep(1)  # Adjust delay as necessary for servos to reach position

                self.set_servo_angle(self.pwm_1, 170)
                self.set_servo_angle(self.pwm_2, 0)
                self.set_servo_angle(self.pwm_3, 170)
                self.set_servo_angle(self.pwm_4, 0)
                time.sleep(1)  # Adjust delay as necessary for servos to reach position

        except KeyboardInterrupt:
            print("Program stopped by user")

        finally:
            self.cleanup()

    def cleanup(self):
        self.pwm_1.stop()
        self.pwm_2.stop()
        self.pwm_3.stop()
        self.pwm_4.stop()
        GPIO.cleanup()

