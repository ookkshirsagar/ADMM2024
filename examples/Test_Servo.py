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


# Initial angles
initial_angle_1 = 180
initial_angle_2 = 0

try:
    # Set the first servo to 160 degrees and the second to 0 degrees before starting the loop
    set_servo_angle(pwm_1, initial_angle_1)
    set_servo_angle(pwm_2, initial_angle_2)
    time.sleep(1)  # Give time for servos to reach the initial positions

    while True:
        # Move the first servo from 160 to 0 degrees and the second from 0 to 160 degrees
        for angle_1, angle_2 in zip(range(180, 159, -10), range(0, 21, 10)):
            set_servo_angle(pwm_1, angle_1)
            set_servo_angle(pwm_2, angle_2)
            time.sleep(0.3)  # Adjust delay as necessary for servos to reach position

        # Move the first servo from 0 to 160 degrees and the second from 160 to 0 degrees
        for angle_1, angle_2 in zip(range(160, 181, 10), range(20, -1, -10)):
            set_servo_angle(pwm_1, angle_1)
            set_servo_angle(pwm_2, angle_2)
            time.sleep(0.3)  # Adjust delay as necessary for servos to reach position

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    pwm_1.stop()
    pwm_2.stop()
    GPIO.cleanup()
