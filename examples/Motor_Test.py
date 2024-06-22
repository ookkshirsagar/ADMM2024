#!/home/admm2024/admm/bin/python

import time
import RPi.GPIO as GPIO

# Define GPIO pins connected to Motor Driver 2 boards 
motor1_enable_pin = 18
motor1_input1_pin = 23
motor1_input2_pin = 24

motor2_enable_pin = 25
motor2_input1_pin = 8
motor2_input2_pin = 12

motor3_enable_pin = 13
motor3_input1_pin = 16
motor3_input2_pin = 20

motor4_enable_pin = 19
motor4_input1_pin = 21
motor4_input2_pin = 26

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor1_enable_pin, GPIO.OUT)
GPIO.setup(motor1_input1_pin, GPIO.OUT)
GPIO.setup(motor1_input2_pin, GPIO.OUT)

GPIO.setup(motor2_enable_pin, GPIO.OUT)
GPIO.setup(motor2_input1_pin, GPIO.OUT)
GPIO.setup(motor2_input2_pin, GPIO.OUT)

GPIO.setup(motor3_enable_pin, GPIO.OUT)
GPIO.setup(motor3_input1_pin, GPIO.OUT)
GPIO.setup(motor3_input2_pin, GPIO.OUT)

GPIO.setup(motor4_enable_pin, GPIO.OUT)
GPIO.setup(motor4_input1_pin, GPIO.OUT)
GPIO.setup(motor4_input2_pin, GPIO.OUT)

# Function to control motor movements
def move_forward():
    GPIO.output(motor1_input1_pin, GPIO.HIGH)
    GPIO.output(motor1_input2_pin, GPIO.LOW)
    GPIO.output(motor1_enable_pin, GPIO.HIGH)

    GPIO.output(motor2_input1_pin, GPIO.HIGH)
    GPIO.output(motor2_input2_pin, GPIO.LOW)
    GPIO.output(motor2_enable_pin, GPIO.HIGH)

    GPIO.output(motor3_input1_pin, GPIO.HIGH)
    GPIO.output(motor3_input2_pin, GPIO.LOW)
    GPIO.output(motor3_enable_pin, GPIO.HIGH)

    GPIO.output(motor4_input1_pin, GPIO.HIGH)
    GPIO.output(motor4_input2_pin, GPIO.LOW)
    GPIO.output(motor4_enable_pin, GPIO.HIGH)

def move_backward():
    GPIO.output(motor1_input1_pin, GPIO.LOW)
    GPIO.output(motor1_input2_pin, GPIO.HIGH)
    GPIO.output(motor1_enable_pin, GPIO.HIGH)

    GPIO.output(motor2_input1_pin, GPIO.LOW)
    GPIO.output(motor2_input2_pin, GPIO.HIGH)
    GPIO.output(motor2_enable_pin, GPIO.HIGH)

    GPIO.output(motor3_input1_pin, GPIO.LOW)
    GPIO.output(motor3_input2_pin, GPIO.HIGH)
    GPIO.output(motor3_enable_pin, GPIO.HIGH)

    GPIO.output(motor4_input1_pin, GPIO.LOW)
    GPIO.output(motor4_input2_pin, GPIO.HIGH)
    GPIO.output(motor4_enable_pin, GPIO.HIGH)

def turn_left():
    # Stop right motors, move left motors
    GPIO.output(motor1_enable_pin, GPIO.LOW)
    GPIO.output(motor2_enable_pin, GPIO.LOW)
    
    GPIO.output(motor3_input1_pin, GPIO.LOW)
    GPIO.output(motor3_input2_pin, GPIO.HIGH)
    GPIO.output(motor3_enable_pin, GPIO.HIGH)
    
    GPIO.output(motor4_input1_pin, GPIO.LOW)
    GPIO.output(motor4_input2_pin, GPIO.HIGH)
    GPIO.output(motor4_enable_pin, GPIO.HIGH)

def turn_right():
    # Stop left motors, move right motors
    GPIO.output(motor3_enable_pin, GPIO.LOW)
    GPIO.output(motor4_enable_pin, GPIO.LOW)
    
    GPIO.output(motor1_input1_pin, GPIO.LOW)
    GPIO.output(motor1_input2_pin, GPIO.HIGH)
    GPIO.output(motor1_enable_pin, GPIO.HIGH)
    
    GPIO.output(motor2_input1_pin, GPIO.LOW)
    GPIO.output(motor2_input2_pin, GPIO.HIGH)
    GPIO.output(motor2_enable_pin, GPIO.HIGH)

def stop_motors():
    GPIO.output(motor1_enable_pin, GPIO.LOW)
    GPIO.output(motor2_enable_pin, GPIO.LOW)
    GPIO.output(motor3_enable_pin, GPIO.LOW)
    GPIO.output(motor4_enable_pin, GPIO.LOW)

try:
    while True:
        # Move forward for 2 seconds
        move_forward()
        time.sleep(2)

        # Stop for 1 second
        stop_motors()
        time.sleep(1)

        # Move backward for 2 seconds
        move_backward()
        time.sleep(2)

        # Stop for 1 second
        stop_motors()
        time.sleep(1)

        # Turn  Left for 2 Seconds
        turn_left()
        time.sleep(2)

        # Stop for 1 second
        stop_motors()
        time.sleep(1)

         # Turn Right for 2 Seconds
        turn_right()
        time.sleep(2)

        # Stop for 1 second
        stop_motors()
        time.sleep(1)


except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    # Clean up GPIO resources
    GPIO.cleanup()
