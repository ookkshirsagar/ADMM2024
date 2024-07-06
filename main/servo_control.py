import RPi.GPIO as GPIO
import time
import threading

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Set GPIO pins for the servos
servo_pin_1 = 17  # GPIO pin for the first servo
servo_pin_2 = 27  # GPIO pin for the second servo
servo_pin_3 = 22  # GPIO pin for the third servo
servo_pin_4 = 4   # GPIO pin for the fourth servo

# Set PWM parameters
GPIO.setup(servo_pin_1, GPIO.OUT)
GPIO.setup(servo_pin_2, GPIO.OUT)
GPIO.setup(servo_pin_3, GPIO.OUT)
GPIO.setup(servo_pin_4, GPIO.OUT)

# Frequency is 50 Hz
pwm1 = GPIO.PWM(servo_pin_1, 50)
pwm2 = GPIO.PWM(servo_pin_2, 50)
pwm3 = GPIO.PWM(servo_pin_3, 50)
pwm4 = GPIO.PWM(servo_pin_4, 50)

# Start PWM
pwm1.start(0)
pwm2.start(0)
pwm3.start(0)
pwm4.start(0)

def move_servo1(angle):
    duty = angle / 18 + 2
    GPIO.output(servo_pin_1, True)
    pwm1.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo_pin_1, False)
    pwm1.ChangeDutyCycle(0)

def move_servo2(angle):
    duty = angle / 18 + 2
    GPIO.output(servo_pin_2, True)
    pwm2.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo_pin_2, False)
    pwm2.ChangeDutyCycle(0)

def move_servo3(angle):
    duty = angle / 18 + 2
    GPIO.output(servo_pin_3, True)
    pwm3.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo_pin_3, False)
    pwm3.ChangeDutyCycle(0)

def move_servo4(angle):
    duty = angle / 18 + 2
    GPIO.output(servo_pin_4, True)
    pwm4.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo_pin_4, False)
    pwm4.ChangeDutyCycle(0)

def main():
    try:
        while True:
            move_servo1(90)  # Move servo 1 to 90 degrees
            time.sleep(30)

            move_servo2(45)  # Move servo 2 to 45 degrees
            time.sleep(30)

            move_servo3(135)  # Move servo 3 to 135 degrees
            time.sleep(30)

            move_servo4(180)  # Move servo 4 to 180 degrees
            time.sleep(30)

    except KeyboardInterrupt:
        print("\nExiting program.")

    finally:
        pwm1.stop()
        pwm2.stop()
        pwm3.stop()
        pwm4.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
