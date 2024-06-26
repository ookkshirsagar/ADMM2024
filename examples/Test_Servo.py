import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Set GPIO pin for the servo
servo_pin = 17  # Change this to the GPIO pin you're using

# Set PWM parameters
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # PWM frequency set to 50 Hz (default)

# Start PWM with 0% duty cycle
pwm.start(0)

try:
    while True:
        # Move from 0 to 180 degrees
        for angle in range(0, 181, 10):
            pwm.ChangeDutyCycle(2.5 + angle / 18)
            time.sleep(0.3)  # Adjust delay as necessary for servo to reach position

        # Move from 180 to 0 degrees
        for angle in range(180, -1, -10):
            pwm.ChangeDutyCycle(2.5 + angle / 18)
            time.sleep(0.3)  # Adjust delay as necessary for servo to reach position

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    pwm.stop()
    GPIO.cleanup()
