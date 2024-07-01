import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Set GPIO pin for the servo
servo1_pin = 17  # Change this to the GPIO pin you're using
servo2_pin = 27

# Set PWM parameters
GPIO.setup(servo1_pin, GPIO.OUT)
GPIO.setup(servo2_pin, GPIO.OUT)

# PWM frequency set to 50 Hz (default)
pwm1 = GPIO.PWM(servo1_pin, 50) 
pwm2 = GPIO.PWM(servo2_pin, 50) 

# Start PWM with 0% duty cycle
pwm1.start(0)
pwm2.start(0)

try:
    while True:
        # Move from 0 to 180 degrees
        for angle in range(0, 181, 10):
            pwm1.ChangeDutyCycle(2.5 + angle / 18)
            pwm2.ChangeDutyCycle(2.5 + angle / 18)
            time.sleep(0.3)  # Adjust delay as necessary for servo to reach position

        # Move from 180 to 0 degrees
        for angle in range(180, -1, -10):
            pwm1.ChangeDutyCycle(2.5 + angle / 18)
            pwm2.ChangeDutyCycle(2.5 + angle / 18)
            time.sleep(0.3)  # Adjust delay as necessary for servo to reach position

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    pwm.stop()
    GPIO.cleanup()
