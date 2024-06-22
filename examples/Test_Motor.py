#!/home/admm2024/admm/bin/python

import RPi.GPIO as GPIO
from time import sleep

# Define GPIO pins for motor control
# Motor Driver 1 (Right Side)
in1_motor1 = 23
in2_motor1 = 24
en_a_motor1 = 18

in3_motor1 = 25
in4_motor1 = 8
en_b_motor1 = 23

# Motor Driver 2 (Left Side)
in1_motor2 = 16
in2_motor2 = 20
en_a_motor2 = 13

in3_motor2 = 21
in4_motor2 = 26
en_b_motor2 = 19

# GPIO setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Motor Driver 1 GPIO setup
GPIO.setup(in1_motor1, GPIO.OUT)
GPIO.setup(in2_motor1, GPIO.OUT)
GPIO.setup(en_a_motor1, GPIO.OUT)
GPIO.setup(in3_motor1, GPIO.OUT)
GPIO.setup(in4_motor1, GPIO.OUT)
GPIO.setup(en_b_motor1, GPIO.OUT)

# Motor Driver 2 GPIO setup
GPIO.setup(in1_motor2, GPIO.OUT)
GPIO.setup(in2_motor2, GPIO.OUT)
GPIO.setup(en_a_motor2, GPIO.OUT)
GPIO.setup(in3_motor2, GPIO.OUT)
GPIO.setup(in4_motor2, GPIO.OUT)
GPIO.setup(en_b_motor2, GPIO.OUT)

# PWM setup
pwm_motor1 = GPIO.PWM(en_a_motor1, 100)
pwm_motor1.start(0)  # Initial PWM duty cycle (0%)

pwm_motor2 = GPIO.PWM(en_a_motor2, 100)
pwm_motor2.start(0)  # Initial PWM duty cycle (0%)

# Function to stop all motors
def stop_motors():
    GPIO.output(in1_motor1, GPIO.LOW)
    GPIO.output(in2_motor1, GPIO.LOW)
    GPIO.output(in3_motor1, GPIO.LOW)
    GPIO.output(in4_motor1, GPIO.LOW)
    GPIO.output(in1_motor2, GPIO.LOW)
    GPIO.output(in2_motor2, GPIO.LOW)
    GPIO.output(in3_motor2, GPIO.LOW)
    GPIO.output(in4_motor2, GPIO.LOW)

# Function to move motors forward
def move_forward():
    GPIO.output(in1_motor1, GPIO.HIGH)
    GPIO.output(in2_motor1, GPIO.LOW)
    GPIO.output(in3_motor1, GPIO.HIGH)
    GPIO.output(in4_motor1, GPIO.LOW)
    GPIO.output(in1_motor2, GPIO.HIGH)
    GPIO.output(in2_motor2, GPIO.LOW)
    GPIO.output(in3_motor2, GPIO.HIGH)
    GPIO.output(in4_motor2, GPIO.LOW)

# Function to move motors backward
def move_backward():
    GPIO.output(in1_motor1, GPIO.LOW)
    GPIO.output(in2_motor1, GPIO.HIGH)
    GPIO.output(in3_motor1, GPIO.LOW)
    GPIO.output(in4_motor1, GPIO.HIGH)
    GPIO.output(in1_motor2, GPIO.LOW)
    GPIO.output(in2_motor2, GPIO.HIGH)
    GPIO.output(in3_motor2, GPIO.LOW)
    GPIO.output(in4_motor2, GPIO.HIGH)

# Function to turn left
def turn_left():
    GPIO.output(in1_motor1, GPIO.HIGH)
    GPIO.output(in2_motor1, GPIO.LOW)
    GPIO.output(in3_motor1, GPIO.LOW)
    GPIO.output(in4_motor1, GPIO.LOW)
    GPIO.output(in1_motor2, GPIO.LOW)
    GPIO.output(in2_motor2, GPIO.LOW)
    GPIO.output(in3_motor2, GPIO.HIGH)
    GPIO.output(in4_motor2, GPIO.LOW)

# Function to turn right
def turn_right():
    GPIO.output(in1_motor1, GPIO.LOW)
    GPIO.output(in2_motor1, GPIO.LOW)
    GPIO.output(in3_motor1, GPIO.HIGH)
    GPIO.output(in4_motor1, GPIO.LOW)
    GPIO.output(in1_motor2, GPIO.HIGH)
    GPIO.output(in2_motor2, GPIO.LOW)
    GPIO.output(in3_motor2, GPIO.LOW)
    GPIO.output(in4_motor2, GPIO.LOW)

try:
    while True:
        user_input = input("Enter command (w: forward, s: backward, a: left, d: right, c: stop): ")

        if user_input == 'w':
            move_forward()
            print("Moving forward")
        
        elif user_input == 's':
            move_backward()
            print("Moving backward")
        
        elif user_input == 'a':
            turn_left()
            print("Turning left")
        
        elif user_input == 'd':
            turn_right()
            print("Turning right")
        
        elif user_input == 'c':
            stop_motors()
            print("Motors stopped")
        
        else:
            print("Invalid command. Please enter w, s, a, d, or c.")

except KeyboardInterrupt:
    print("\nExiting program.")

finally:
    stop_motors()
    pwm_motor1.stop()
    pwm_motor2.stop()
    GPIO.cleanup()
