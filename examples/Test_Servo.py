import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Set GPIO pin for the servo
servo_pin = 17  # Change this to the pin you've connected your servo to

# Set GPIO pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Set PWM parameters
pwm_frequency = 50  # Frequency in Hz (typical for servos)

# Initialize PWM on the servo pin
pwm = GPIO.PWM(servo_pin, pwm_frequency)
pwm.start(0)  # Initial duty cycle

def set_servo_angle(angle):
    duty_cycle = 2 + (angle / 18)  # Convert angle to duty cycle
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Allow time for the servo to move
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        # Move servo from 0 to 30 degrees
        for angle in range(0, 31, 1):
            set_servo_angle(angle)
            time.sleep(0.05)  # Adjust as necessary for smooth motion

        # Move servo from 30 to 0 degrees
        for angle in range(30, -1, -1):
            set_servo_angle(angle)
            time.sleep(0.05)  # Adjust as necessary for smooth motion

except KeyboardInterrupt:
    print("Program stopped")

finally:
    pwm.stop()
    GPIO.cleanup()
