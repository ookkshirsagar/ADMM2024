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

# MPU6050 sensor address
sensor_address = 0x68  # Check your MPU6050 address

# Create MPU6050 instance
sensor = mpu6050(sensor_address)

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

# Function to set motor speed (0 to 100)
def set_motor_speed(pwm, speed):
    if speed < 0:
        speed = 0
    elif speed > 100:
        speed = 100
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

# Function to turn left using PID control with MPU6050
def turn_left(sensor, gyro_offsets):
    desired_angle = 90.0  # Desired angle to turn
    kp = 0.1  # Proportional gain
    ki = 0.1  # Integral gain
    kd = 0.1  # Derivative gain
    max_speed = 70.0  # Maximum PWM duty cycle
    min_speed = 30.0  # Minimum PWM duty cycle
    dt = 0.01  # Sample time

    current_angle = 0.0
    integrated_error = 0.0
    last_error = 0.0

    while current_angle < desired_angle:
        gyro_data = sensor.get_gyro_data()
        gyro_x = gyro_data['x'] - gyro_offsets[0]
        gyro_y = gyro_data['y'] - gyro_offsets[1]
        gyro_z = gyro_data['z'] - gyro_offsets[2]

        angle_z = gyro_z * dt
        current_angle += angle_z

        # PID calculations
        error = desired_angle - current_angle
        integrated_error += error * dt
        derivative_error = (error - last_error) / dt

        # PID output
        correction = kp * error + ki * integrated_error + kd * derivative_error

        # Adjust motor speeds based on correction
        left_speed = max(min_speed, max_speed - correction)
        right_speed = max(min_speed, max_speed + correction)

        # Ensure speed is within valid range (0 to 100)
        left_speed = min(max(left_speed, 0), 100)
        right_speed = min(max(right_speed, 0), 100)

        # Left motors stop
        GPIO.output(left_front_in1, GPIO.LOW)
        GPIO.output(left_front_in2, GPIO.HIGH)
        GPIO.output(left_rear_in1, GPIO.HIGH)
        GPIO.output(left_rear_in2, GPIO.LOW)

        # Right motors move forward
        GPIO.output(right_front_in1, GPIO.LOW)
        GPIO.output(right_front_in2, GPIO.HIGH)
        GPIO.output(right_rear_in1, GPIO.HIGH)
        GPIO.output(right_rear_in2, GPIO.LOW)

        set_motor_speed(pwm_left_front, left_speed)
        set_motor_speed(pwm_left_rear, left_speed)
        set_motor_speed(pwm_right_front, right_speed)
        set_motor_speed(pwm_right_rear, right_speed)

        # Update last error
        last_error = error

        # Print for debugging (optional)
        print(f"Current Angle: {current_angle:.2f} deg")

        time.sleep(dt)

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

# Function for gyro calibration
def calibrate_gyro(sensor):
    print("Calibrating gyro... Keep the sensor still!")
    gyro_offsets = [0, 0, 0]
    num_samples = 100

    for _ in range(num_samples):
        gyro_data = sensor.get_gyro_data()
        gyro_offsets[0] += gyro_data['x']
        gyro_offsets[1] += gyro_data['y']
        gyro_offsets[2] += gyro_data['z']
        time.sleep(0.01)

    gyro_offsets = [offset / num_samples for offset in gyro_offsets]
    print(f"Gyro offsets: {gyro_offsets}")

    return gyro_offsets

try:
    # Calibrate gyro and get offsets
    gyro_offsets = calibrate_gyro(sensor)

    while True:
        # Move forward for 5 seconds
        move_forward()
        print("Moving forward...")
        time.sleep(5)

        # Stop motors and wait 1 second
        stop_motors()
        time.sleep(1)

        # Turn left using PID control with MPU6050
        turn_left(sensor, gyro_offsets)

        # Stop motors and wait 1 second
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
