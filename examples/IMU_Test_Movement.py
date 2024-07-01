import RPi.GPIO as GPIO
import time
from mpu6050 import mpu6050  # Import the MPU6050 library (ensure you have installed it)

# MPU6050 initialization
mpu = mpu6050(0x68)  # Initialize MPU6050 with the default address 0x68

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

# Function to turn left by a specified angle using MPU6050
def turn_left(angle):
    start_angle = mpu.get_angle()[0]  # Get initial angle from MPU6050
    target_angle = start_angle - angle  # Calculate target angle
    
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
    
    set_motor_speed(pwm_left_front, 100)
    set_motor_speed(pwm_left_rear, 100)
    set_motor_speed(pwm_right_front, 100)
    set_motor_speed(pwm_right_rear, 100)
    
    # Monitor angle until target angle is reached
    current_angle = start_angle
    while current_angle > target_angle:
        current_angle = mpu.get_angle()[0]
        time.sleep(0.1)  # Adjust sleep time as needed

    stop_motors()
    time.sleep(1)  # Wait after turning

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

try:
    while True:
        # Move forward for 5 seconds
        move_forward()
        print("Moving forward...")
        time.sleep(2)
        
        # Stop motors and wait 1 second
        stop_motors()
        time.sleep(1)
        
        # Turn left 90 degrees
        turn_left(90)
        print("Turning left 90 degrees...")
        
        # Stop motors and wait 1 second
        stop_motors()
        time.sleep(1)
        
        # Move forward for a short distance
        move_forward()
        print("Moving forward a short distance...")
        time.sleep(0.2)  # Adjust the time to move a short distance
        
        # Stop motors and wait 1 second
        stop_motors()
        time.sleep(1)
        
        # Turn left again 90 degrees
        turn_left(90)
        print("Turning left again 90 degrees...")
        
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
