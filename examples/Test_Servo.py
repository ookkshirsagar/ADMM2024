import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Set GPIO pin for the servo
servo_pin = 17  # Change this to the GPIO pin you're using

# Set PWM parameters
frequency = 1000  # PWM frequency in Hz
period = 1.0 / frequency  # Period in seconds

# Duty cycle constants for 0 and 180 degrees
duty_cycle_0deg = 2.5  # Adjusted for your servo
duty_cycle_180deg = 12.5  # Adjusted for your servo

# Set GPIO pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Initialize PWM on servo pin
pwm = GPIO.PWM(servo_pin, frequency)
pwm.start(0)  # Start PWM with 0% duty cycle

def set_angle(angle):
    duty_cycle = (duty_cycle_180deg - duty_cycle_0deg) / 180 * angle + duty_cycle_0deg
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.3)  # Adjust delay as necessary for servo to reach position

try:
    while True:
        # Move from 0 to 180 degrees
        for angle in range(0, 161, 5):
            set_angle(angle)

        # Move from 180 to 0 degrees
        for angle in range(160, -1, -5):
            set_angle(angle)

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    pwm.stop()
    GPIO.cleanup()
