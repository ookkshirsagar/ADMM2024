import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Set GPIO pins for the servos
servo_pin_1 = 17  # GPIO pin for the first servo
servo_pin_2 = 27  # GPIO pin for the second servo

# Set PWM parameters
GPIO.setup(servo_pin_1, GPIO.OUT)
GPIO.setup(servo_pin_2, GPIO.OUT)
pwm_1 = GPIO.PWM(servo_pin_1, 50)  # PWM frequency set to 50 Hz for the first servo
pwm_2 = GPIO.PWM(servo_pin_2, 50)  # PWM frequency set to 50 Hz for the second servo

# Start PWM with 0% duty cycle
pwm_1.start(0)
pwm_2.start(0)

# Function to set servo angle
def set_servo_angle(pwm, angle):
    duty_cycle = 2.5 + (angle / 18.0)  # Adjust the duty cycle calculation if needed
    pwm.ChangeDutyCycle(duty_cycle)

# Initial angle variable
initial_angle = 0

try:
    # Set both servos to the initial angle before starting the loop
    set_servo_angle(pwm_1, initial_angle)
    set_servo_angle(pwm_2, initial_angle)
    time.sleep(1)  # Give time for servos to reach the initial position

    while True:
        # Move from initial angle to 160 degrees
        for angle in range(initial_angle, 161, 10):
            set_servo_angle(pwm_1, angle)
            set_servo_angle(pwm_2, angle)
            time.sleep(0.3)  # Adjust delay as necessary for servos to reach position

        # Move from 160 to initial angle degrees
        for angle in range(160, initial_angle - 1, -10):
            set_servo_angle(pwm_1, angle)
            set_servo_angle(pwm_2, angle)
            time.sleep(0.3)  # Adjust delay as necessary for servos to reach position

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    pwm_1.stop()
    pwm_2.stop()
    GPIO.cleanup()
