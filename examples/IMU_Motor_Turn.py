#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
from mpu6050 import mpu6050

# Motor Driver 1 Pins (Left Motors)
left_front_in1 = 23
left_front_in2 = 24
left_front_en = 18
left_rear_in1 = 25
left_rear_in2 = 8
left_rear_en = 12

# Motor Driver 2 Pins (Right Motors)
right_front_in1 = 16
right_front_in2 = 20
right_front_en = 13
right_rear_in1 = 21
right_rear_in2 = 26
right_rear_en = 19

# Initialize MPU6050 sensor
mpu6050_address = 0x68  # Replace with your MPU6050 I2C address
sensor = mpu6050(mpu6050_address)

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set up Left Motors (Motor Driver 1)
GPIO.setup(left_front_in1, GPIO.OUT)
GPIO.setup(left_front_in2, GPIO.OUT)
GPIO.setup(left_front_en, GPIO.OUT)
GPIO.setup(left_rear_in1, GPIO.OUT)
GPIO.setup(left_rear_in2, GPIO.OUT)
GPIO.setup(left_rear_en, GPIO.OUT)

# Set up Right Motors (Motor Driver 2)
GPIO.setup(right_front_in1, GPIO.OUT)
GPIO.setup(right_front_in2, GPIO.OUT)
GPIO.setup(right_front_en, GPIO.OUT)
GPIO.setup(right_rear_in1, GPIO.OUT)
GPIO.setup(right_rear_in2, GPIO.OUT)
GPIO.setup(right_rear_en, GPIO.OUT)

# Set up PWM for motor speed control
pwm_left_front = GPIO.PWM(left_front_en, 100)
pwm_left_rear = GPIO.PWM(left_rear_en, 100)
pwm_right_front = GPIO.PWM(right_front_en, 100)
pwm_right_rear = GPIO.PWM(right_rear_en, 100)

# Start PWM with a duty cycle of 0 (motors off)
pwm_left_front.start(0)
pwm_left_rear.start(0)
pwm_right_front.start(0)
pwm_right_rear.start(0)

# Function to set motor speed
def set_motor_speed(pwm, speed):
    pwm.ChangeDutyCycle(speed)

# Function to move motors forward
def move_forward():
    # Left motors move forward
    GPIO.output(left_front_in1, GPIO.HIGH)
    GPIO.output(left_front_in2, GPIO.LOW)
    GPIO.output(left_rear_in1, GPIO.LOW)
    GPIO.output(left_rear_in2, GPIO.HIGH)
    
    # Right motors move forward
    GPIO.output(right_front_in1, GPIO.LOW)
    GPIO.output(right_front_in2, GPIO.HIGH)
    GPIO.output(right_rear_in1, GPIO.HIGH)
    GPIO.output(right_rear_in2, GPIO.LOW)
    
    set_motor_speed(pwm_left_front, 100)
    set_motor_speed(pwm_left_rear, 100)
    set_motor_speed(pwm_right_front, 100)
    set_motor_speed(pwm_right_rear, 100)

# Function to move motors backward
def move_backward():
    # Left motors move backward
    GPIO.output(left_front_in1, GPIO.LOW)
    GPIO.output(left_front_in2, GPIO.HIGH)
    GPIO.output(left_rear_in1, GPIO.HIGH)
    GPIO.output(left_rear_in2, GPIO.LOW)
    
    # Right motors move backward
    GPIO.output(right_front_in1, GPIO.HIGH)
    GPIO.output(right_front_in2, GPIO.LOW)
    GPIO.output(right_rear_in1, GPIO.LOW)
    GPIO.output(right_rear_in2, GPIO.HIGH)
    
    set_motor_speed(pwm_left_front, 100)
    set_motor_speed(pwm_left_rear, 100)
    set_motor_speed(pwm_right_front, 100)
    set_motor_speed(pwm_right_rear, 100)

# Function to turn left (approximately 180 degrees)
def turn_left():
    target_angle = 180.0
    kp = 2.0  # Proportional gain for angular velocity control
    max_speed = 70  # Maximum PWM duty cycle for motors
    min_speed = 30  # Minimum PWM duty cycle for motors
    
    initial_angle = sensor.get_angle()
    current_angle = initial_angle
    last_time = time.time()
    
    while current_angle - initial_angle < target_angle:
        dt = time.time() - last_time
        angular_velocity = sensor.get_gyro_data()['y']  # Adjust axis as per your setup
        current_angle += angular_velocity * dt
        
        error = target_angle - (current_angle - initial_angle)
        speed = kp * error
        
        # Adjust motor speeds based on error
        left_speed = min(max_speed, max(min_speed, 50 + speed))
        right_speed = min(max_speed, max(min_speed, 50 - speed))
        
        # Left motors move backward
        GPIO.output(left_front_in1, GPIO.LOW)
        GPIO.output(left_front_in2, GPIO.HIGH)
        GPIO.output(left_rear_in1, GPIO.HIGH)
        GPIO.output(left_rear_in2, GPIO.LOW)
        set_motor_speed(pwm_left_front, left_speed)
        set_motor_speed(pwm_left_rear, left_speed)
        
        # Right motors move forward
        GPIO.output(right_front_in1, GPIO.LOW)
        GPIO.output(right_front_in2, GPIO.HIGH)
        GPIO.output(right_rear_in1, GPIO.HIGH)
        GPIO.output(right_rear_in2, GPIO.LOW)
        set_motor_speed(pwm_right_front, right_speed)
        set_motor_speed(pwm_right_rear, right_speed)
        
        last_time = time.time()
    
    stop_motors()

# Function to turn right (approximately 180 degrees)
def turn_right():
    target_angle = -180.0  # Negative because turning right
    kp = 2.0  # Proportional gain for angular velocity control
    max_speed = 70  # Maximum PWM duty cycle for motors
    min_speed = 30  # Minimum PWM duty cycle for motors
    
    initial_angle = sensor.get_angle()
    current_angle = initial_angle
    last_time = time.time()
    
    while current_angle - initial_angle > target_angle:
        dt = time.time() - last_time
        angular_velocity = sensor.get_gyro_data()['y']  # Adjust axis as per your setup
        current_angle += angular_velocity * dt
        
        error = target_angle - (current_angle - initial_angle)
        speed = kp * error
        
        # Adjust motor speeds based on error
        left_speed = min(max_speed, max(min_speed, 50 - speed))
        right_speed = min(max_speed, max(min_speed, 50 + speed))
        
        # Left motors move forward
        GPIO.output(left_front_in1, GPIO.HIGH)
        GPIO.output(left_front_in2, GPIO.LOW)
        GPIO.output(left_rear_in1, GPIO.LOW)
        GPIO.output(left_rear_in2, GPIO.HIGH)
        set_motor_speed(pwm_left_front, left_speed)
        set_motor_speed(pwm_left_rear, left_speed)
        
        # Right motors move backward
        GPIO.output(right_front_in1, GPIO.HIGH)
        GPIO.output(right_front_in2, GPIO.LOW)
        GPIO.output(right_rear_in1, GPIO.LOW)
        GPIO.output(right_rear_in2, GPIO.HIGH)
        set_motor_speed(pwm_right_front, right_speed)
        set_motor_speed(pwm_right_rear, right_speed)
        
        last_time = time.time()
    
    stop_motors()

# Function to stop all motors
def stop_motors():
    GPIO.output(left_front_in1, GPIO.LOW)
    GPIO.output(left_front_in2, GPIO.LOW)
    GPIO.output(left_rear_in1, GPIO.LOW)
    GPIO.output(left_rear_in2, GPIO.LOW)
    GPIO.output(right_front_in1, GPIO.LOW)
    GPIO.output(right_front_in2, GPIO.LOW)
    GPIO.output(right_rear_in1, GPIO.LOW)
    GPIO.output(right_rear_in2, GPIO.LOW)
    set_motor_speed(pwm_left_front, 0)
    set_motor_speed(pwm_left_rear, 0)
    set_motor_speed(pwm_right_front, 0)
    set_motor_speed(pwm_right_rear, 0)

# Function to calibrate and tare MPU6050 sensor
def calibrate_and_tare():
    print("Calibrating and taring MPU6050 sensor...")
    sensor.calibrate()
    sensor.tare()
    print("Calibration and taring completed.")
    time.sleep(2)  # Adjust as needed

try:
    calibrate_and_tare()

    while True:
        # Move forward for 5 seconds, then stop for 1 second
        move_forward()
        print("Forward")
        time.sleep(5)
        stop_motors()
        time.sleep(1)
        
        # Move backward for 5 seconds, then stop for 1 second
        move_backward()
        print("Backward")
        time.sleep(5)
        stop_motors()
        time.sleep(1)
        
        # Turn left for approximately 180 degrees, then stop for 1 second
        turn_left()
        print("Left")
        time.sleep(1)  # Adjust duration for precise turn
        stop_motors()
        time.sleep(1)
        
        # Turn right for approximately 180 degrees, then stop for 1 second
        turn_right()
        print("Right")
        time.sleep(1)  # Adjust duration for precise turn
        stop_motors()
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    # Clean up GPIO resources
    pwm_left_front.stop()
    pwm_left_rear.stop()
    pwm_right_front.stop()
    pwm_right_rear.stop()
    GPIO.cleanup()

