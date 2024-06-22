#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

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

def setup():
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
    global pwm_left_front, pwm_left_rear, pwm_right_front, pwm_right_rear
    pwm_left_front = GPIO.PWM(left_front_en, 1000)
    pwm_left_rear = GPIO.PWM(left_rear_en, 1000)
    pwm_right_front = GPIO.PWM(right_front_en, 1000)
    pwm_right_rear = GPIO.PWM(right_rear_en, 1000)

    # Start PWM with a duty cycle of 0 (motors off)
    pwm_left_front.start(0)
    pwm_left_rear.start(0)
    pwm_right_front.start(0)
    pwm_right_rear.start(0)

    print("GPIO setup complete.")

def set_motor_speed(pwm, speed):
    pwm.ChangeDutyCycle(speed)
    print(f"Set motor speed to {speed}%")

def move_forward():
    print("Moving forward...")
    GPIO.output(left_front_in1, GPIO.HIGH)
    GPIO.output(left_front_in2, GPIO.LOW)
    GPIO.output(left_rear_in1, GPIO.HIGH)
    GPIO.output(left_rear_in2, GPIO.LOW)
    GPIO.output(right_front_in1, GPIO.HIGH)
    GPIO.output(right_front_in2, GPIO.LOW)
    GPIO.output(right_rear_in1, GPIO.HIGH)
    GPIO.output(right_rear_in2, GPIO.LOW)
    set_motor_speed(pwm_left_front, 100)
    set_motor_speed(pwm_left_rear, 100)
    set_motor_speed(pwm_right_front, 100)
    set_motor_speed(pwm_right_rear, 100)

def move_backward():
    print("Moving backward...")
    GPIO.output(left_front_in1, GPIO.LOW)
    GPIO.output(left_front_in2, GPIO.HIGH)
    GPIO.output(left_rear_in1, GPIO.LOW)
    GPIO.output(left_rear_in2, GPIO.HIGH)
    GPIO.output(right_front_in1, GPIO.LOW)
    GPIO.output(right_front_in2, GPIO.HIGH)
    GPIO.output(right_rear_in1, GPIO.LOW)
    GPIO.output(right_rear_in2, GPIO.HIGH)
    set_motor_speed(pwm_left_front, 100)
    set_motor_speed(pwm_left_rear, 100)
    set_motor_speed(pwm_right_front, 100)
    set_motor_speed(pwm_right_rear, 100)

def turn_left():
    print("Turning left...")
    GPIO.output(left_front_in1, GPIO.LOW)
    GPIO.output(left_front_in2, GPIO.HIGH)
    GPIO.output(left_rear_in1, GPIO.LOW)
    GPIO.output(left_rear_in2, GPIO.HIGH)
    GPIO.output(right_front_in1, GPIO.HIGH)
    GPIO.output(right_front_in2, GPIO.LOW)
    GPIO.output(right_rear_in1, GPIO.HIGH)
    GPIO.output(right_rear_in2, GPIO.LOW)
    set_motor_speed(pwm_left_front, 100)
    set_motor_speed(pwm_left_rear, 100)
    set_motor_speed(pwm_right_front, 100)
    set_motor_speed(pwm_right_rear, 100)

def turn_right():
    print("Turning right...")
    GPIO.output(left_front_in1, GPIO.HIGH)
    GPIO.output(left_front_in2, GPIO.LOW)
    GPIO.output(left_rear_in1, GPIO.HIGH)
    GPIO.output(left_rear_in2, GPIO.LOW)
    GPIO.output(right_front_in1, GPIO.LOW)
    GPIO.output(right_front_in2, GPIO.HIGH)
    GPIO.output(right_rear_in1, GPIO.LOW)
    GPIO.output(right_rear_in2, GPIO.HIGH)
    set_motor_speed(pwm_left_front, 100)
    set_motor_speed(pwm_left_rear, 100)
    set_motor_speed(pwm_right_front, 100)
    set_motor_speed(pwm_right_rear, 100)

def stop_motors():
    print("Stopping motors...")
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

def main():
    setup()
    try:
        while True:
            move_forward()
            time.sleep(2)
            stop_motors()
            time.sleep(1)
            move_backward()
            time.sleep(2)
            stop_motors()
            time.sleep(1)
            turn_left()
            time.sleep(2)
            stop_motors()
            time.sleep(1)
            turn_right()
            time.sleep(2)
            stop_motors()
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting program.")

    finally:
        print("Cleaning up GPIO...")
        pwm_left_front.stop()
        pwm_left_rear.stop()
        pwm_right_front.stop()
        pwm_right_rear.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
