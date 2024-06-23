#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

# Define motor driver pin numbers
# Motor Driver 1 Pins (Left Motors)
LEFT_FRONT_IN1 = 23
LEFT_FRONT_IN2 = 24
LEFT_FRONT_EN = 18
LEFT_REAR_IN1 = 25
LEFT_REAR_IN2 = 8
LEFT_REAR_EN = 12

# Motor Driver 2 Pins (Right Motors)
RIGHT_FRONT_IN1 = 16
RIGHT_FRONT_IN2 = 20
RIGHT_FRONT_EN = 13
RIGHT_REAR_IN1 = 21
RIGHT_REAR_IN2 = 26
RIGHT_REAR_EN = 19

# Define constants for PWM frequency and motor speed
PWM_FREQUENCY = 100
MOTOR_SPEED = 100  # Percentage of maximum speed

# Define turn duration (in seconds) for approximately 180-degree turn
TURN_DURATION = 1.8

# GPIO setup
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Set up Left Motors (Motor Driver 1)
    GPIO.setup(LEFT_FRONT_IN1, GPIO.OUT)
    GPIO.setup(LEFT_FRONT_IN2, GPIO.OUT)
    GPIO.setup(LEFT_FRONT_EN, GPIO.OUT)
    GPIO.setup(LEFT_REAR_IN1, GPIO.OUT)
    GPIO.setup(LEFT_REAR_IN2, GPIO.OUT)
    GPIO.setup(LEFT_REAR_EN, GPIO.OUT)

    # Set up Right Motors (Motor Driver 2)
    GPIO.setup(RIGHT_FRONT_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_FRONT_IN2, GPIO.OUT)
    GPIO.setup(RIGHT_FRONT_EN, GPIO.OUT)
    GPIO.setup(RIGHT_REAR_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_REAR_IN2, GPIO.OUT)
    GPIO.setup(RIGHT_REAR_EN, GPIO.OUT)

# Initialize PWM for motor speed control
def init_pwm():
    pwm_left_front = GPIO.PWM(LEFT_FRONT_EN, PWM_FREQUENCY)
    pwm_left_rear = GPIO.PWM(LEFT_REAR_EN, PWM_FREQUENCY)
    pwm_right_front = GPIO.PWM(RIGHT_FRONT_EN, PWM_FREQUENCY)
    pwm_right_rear = GPIO.PWM(RIGHT_REAR_EN, PWM_FREQUENCY)

    pwm_left_front.start(0)
    pwm_left_rear.start(0)
    pwm_right_front.start(0)
    pwm_right_rear.start(0)

    return pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear

# Set motor speed
def set_motor_speed(pwm, speed):
    pwm.ChangeDutyCycle(speed)

# Move motors forward
def move_forward(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear):
    GPIO.output(LEFT_FRONT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_FRONT_IN2, GPIO.LOW)
    GPIO.output(LEFT_REAR_IN1, GPIO.LOW)
    GPIO.output(LEFT_REAR_IN2, GPIO.HIGH)

    GPIO.output(RIGHT_FRONT_IN1, GPIO.LOW)
    GPIO.output(RIGHT_FRONT_IN2, GPIO.HIGH)
    GPIO.output(RIGHT_REAR_IN1, GPIO.HIGH)
    GPIO.output(RIGHT_REAR_IN2, GPIO.LOW)

    set_motor_speed(pwm_left_front, MOTOR_SPEED)
    set_motor_speed(pwm_left_rear, MOTOR_SPEED)
    set_motor_speed(pwm_right_front, MOTOR_SPEED)
    set_motor_speed(pwm_right_rear, MOTOR_SPEED)

# Move motors backward
def move_backward(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear):
    GPIO.output(LEFT_FRONT_IN1, GPIO.LOW)
    GPIO.output(LEFT_FRONT_IN2, GPIO.HIGH)
    GPIO.output(LEFT_REAR_IN1, GPIO.HIGH)
    GPIO.output(LEFT_REAR_IN2, GPIO.LOW)

    GPIO.output(RIGHT_FRONT_IN1, GPIO.HIGH)
    GPIO.output(RIGHT_FRONT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_REAR_IN1, GPIO.LOW)
    GPIO.output(RIGHT_REAR_IN2, GPIO.HIGH)

    set_motor_speed(pwm_left_front, MOTOR_SPEED)
    set_motor_speed(pwm_left_rear, MOTOR_SPEED)
    set_motor_speed(pwm_right_front, MOTOR_SPEED)
    set_motor_speed(pwm_right_rear, MOTOR_SPEED)

# Stop all motors
def stop_motors(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear):
    GPIO.output(LEFT_FRONT_IN1, GPIO.LOW)
    GPIO.output(LEFT_FRONT_IN2, GPIO.LOW)
    GPIO.output(LEFT_REAR_IN1, GPIO.LOW)
    GPIO.output(LEFT_REAR_IN2, GPIO.LOW)
    
    GPIO.output(RIGHT_FRONT_IN1, GPIO.LOW)
    GPIO.output(RIGHT_FRONT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_REAR_IN1, GPIO.LOW)
    GPIO.output(RIGHT_REAR_IN2, GPIO.LOW)

    set_motor_speed(pwm_left_front, 0)
    set_motor_speed(pwm_left_rear, 0)
    set_motor_speed(pwm_right_front, 0)
    set_motor_speed(pwm_right_rear, 0)

# Turn left approximately 180 degrees
def turn_left(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear):
    move_backward(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)
    time.sleep(TURN_DURATION)
    stop_motors(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)

# Turn right approximately 180 degrees
def turn_right(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear):
    move_forward(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)
    time.sleep(TURN_DURATION)
    stop_motors(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)

# Main function to control the vehicle
def main():
    try:
        setup_gpio()
        pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear = init_pwm()

        while True:
            move_forward(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)
            print("Forward")
            time.sleep(5)
            stop_motors(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)
            time.sleep(1)

            move_backward(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)
            print("Backward")
            time.sleep(5)
            stop_motors(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)
            time.sleep(1)

            turn_left(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)
            print("Left")
            time.sleep(1)
            stop_motors(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)
            time.sleep(1)

            turn_right(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)
            print("Right")
            time.sleep(1)
            stop_motors(pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear)
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

if __name__ == "__main__":
    main()
