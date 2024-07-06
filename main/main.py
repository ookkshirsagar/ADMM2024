import time
import board
import busio
import digitalio
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
import threading
import serial
import struct
import paho.mqtt.client as mqtt
import adafruit_vl53l0x

# Motor Driver Pins (Left Motors)
left_front_in1 = 23
left_front_in2 = 24
left_front_en = 18
left_rear_in1 = 25
left_rear_in2 = 8
left_rear_en = 12

# Motor Driver Pins (Right Motors)
right_front_in1 = 16
right_front_in2 = 20
right_front_en = 13
right_rear_in1 = 21
right_rear_in2 = 26
right_rear_en = 19

# MPU6050 sensor address
sensor_address = 0x68  # Check your MPU6050 address
sensor = mpu6050(sensor_address)

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set up Left Motors
GPIO.setup(left_front_in1, GPIO.OUT)
GPIO.setup(left_front_in2, GPIO.OUT)
GPIO.setup(left_front_en, GPIO.OUT)
GPIO.setup(left_rear_in1, GPIO.OUT)
GPIO.setup(left_rear_in2, GPIO.OUT)
GPIO.setup(left_rear_en, GPIO.OUT)

# Set up Right Motors
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
    if speed < 0:
        speed = 0
    elif speed > 100:
        speed = 100
    pwm.ChangeDutyCycle(speed)

# Function to move forward
def move_forward(speed=20):
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

    set_motor_speed(pwm_left_front, speed)
    set_motor_speed(pwm_left_rear, speed)
    set_motor_speed(pwm_right_front, speed)
    set_motor_speed(pwm_right_rear, speed)
# Function to stop all motors
def stop_motors():
    set_motor_speed(pwm_left_front, 0)
    set_motor_speed(pwm_left_rear, 0)
    set_motor_speed(pwm_right_front, 0)
    set_motor_speed(pwm_right_rear, 0)

    GPIO.output(left_front_in1, GPIO.LOW)
    GPIO.output(left_front_in2, GPIO.LOW)
    GPIO.output(left_rear_in1, GPIO.LOW)
    GPIO.output(left_rear_in2, GPIO.LOW)
    GPIO.output(right_front_in1, GPIO.LOW)
    GPIO.output(right_front_in2, GPIO.LOW)
    GPIO.output(right_rear_in1, GPIO.LOW)
    GPIO.output(right_rear_in2, GPIO.LOW)

# Function to move forward with stop after 2 seconds
def move_forward_with_stop():
    move_forward()
    time.sleep(5)
    stop_motors()

# Function to turn left with gyro control
def turn_left(sensor, angle=82.0, speed=100):
    kp = 1.0
    current_angle = 0.0
    dt = 0.005

    while current_angle < angle:
        start_time = time.time()

        calibrated_gyro = get_calibrated_gyro_data(sensor)
        gyro_z = calibrated_gyro['z']
        angle_z = gyro_z * dt
        current_angle += angle_z

        correction = kp * (angle - current_angle)
        turn_speed = min(max(speed + correction, 30), 100)

        # Left motors backward
        GPIO.output(left_front_in1, GPIO.LOW)
        GPIO.output(left_front_in2, GPIO.HIGH)
        GPIO.output(left_rear_in1, GPIO.HIGH)
        GPIO.output(left_rear_in2, GPIO.LOW)

        # Right motors forward
        GPIO.output(right_front_in1, GPIO.LOW)
        GPIO.output(right_front_in2, GPIO.HIGH)
        GPIO.output(right_rear_in1, GPIO.HIGH)
        GPIO.output(right_rear_in2, GPIO.LOW)

        set_motor_speed(pwm_left_front, turn_speed)
        set_motor_speed(pwm_left_rear, turn_speed)
        set_motor_speed(pwm_right_front, turn_speed)
        set_motor_speed(pwm_right_rear, turn_speed)

        elapsed_time = time.time() - start_time
        if elapsed_time < dt:
            time.sleep(dt - elapsed_time)

    stop_motors()

# Function to turn right with gyro control
def turn_right(sensor, angle=86, speed=100):
    kp = 1.0
    current_angle = 0.0
    dt = 0.005

    # Convert angle to negative for clockwise turn
    angle = -abs(angle)

    while current_angle > angle:  # We use > because angle is negative
        start_time = time.time()

        calibrated_gyro = get_calibrated_gyro_data(sensor)
        gyro_z = calibrated_gyro['z']
        angle_z = gyro_z * dt
        current_angle += angle_z

        correction = kp * (angle - current_angle)
        turn_speed = min(max(speed + correction, 30), 100)

        # Left motors forward
        GPIO.output(left_front_in1, GPIO.HIGH)
        GPIO.output(left_front_in2, GPIO.LOW)
        GPIO.output(left_rear_in1, GPIO.LOW)
        GPIO.output(left_rear_in2, GPIO.HIGH)

        # Right motors backward
        GPIO.output(right_front_in1, GPIO.HIGH)
        GPIO.output(right_front_in2, GPIO.LOW)
        GPIO.output(right_rear_in1, GPIO.LOW)
        GPIO.output(right_rear_in2, GPIO.HIGH)

        set_motor_speed(pwm_left_front, turn_speed)
        set_motor_speed(pwm_left_rear, turn_speed)
        set_motor_speed(pwm_right_front, turn_speed)
        set_motor_speed(pwm_right_rear, turn_speed)

        elapsed_time = time.time() - start_time
        if elapsed_time < dt:
            time.sleep(dt - elapsed_time)

    stop_motors()
# Function to move forward for a specified duration
def move_forward_for_duration(duration=2, speed=20):
    move_forward(speed)
    time.sleep(duration)
    stop_motors()

# Function to handle obstacle avoidance
def avoid_obstacle(sensor):
    stop_motors()
    mqtt_client.publish("robot/status", "Obstacle detected. Stopping and avoiding.")

    # Read TOF sensor distances
    front_distance = sensor['sensor_front'].range
    left_distance = sensor['sensor_left'].range
    right_distance = sensor['sensor_right'].range

    if right_distance < 200:  # If right distance is less than 200mm, turn left
        turn_left(sensor)
    elif left_distance < 200:  # If left distance is less than 200mm, turn right
        turn_right(sensor)
    else:
        # Default behavior if no clear direction to turn
        turn_left(sensor)  # Adjust as needed

# Function to get calibrated gyroscope data
def get_calibrated_gyro_data(sensor):
    gyro_offsets = {
        'x': -0.40,
        'y': 0.50,
        'z': -1.45
    }
    raw_data = sensor.get_gyro_data()
    calibrated_data = {
        'x': raw_data['x'] - gyro_offsets['x'],
        'y': raw_data['y'] - gyro_offsets['y'],
        'z': raw_data['z'] - gyro_offsets['z']
    }
    return calibrated_data

# TOF Sensor Integration

# Number of samples to average for stability
NUM_SAMPLES = 20

# Offset adjustment based on calibration (if needed)
OFFSET = 20  # Adjust this value based on calibration measurements

# Define the XSHUT pins for each sensor
XSHUT_PINS = {
    'sensor_front': board.D5,
    'sensor_left': board.D6,
    'sensor_right': board.D7
}

# New addresses for each sensor
NEW_ADDRESSES = {
    'sensor_front': 0x30,
    'sensor_left': 0x31,
    'sensor_right': 0x32
}

# Exponential Moving Average (EMA) alpha
EMA_ALPHA = 0.5

# Function to initialize sensor with specified address
def initialize_sensor(i2c, xshut_pin, new_address):
    try:
        xshut = digitalio.DigitalInOut(xshut_pin)
        xshut.direction = digitalio.Direction.OUTPUT
        xshut.value = False  # Keep the sensor in reset state

        time.sleep(0.1)
        xshut.value = True  # Bring the sensor out of reset

        # Try to initialize sensor at the default address
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
        
        # Attempt to change the sensor address
        sensor.set_address(new_address)
        print(f"VL53L0X sensor initialized and set to address {hex(new_address)}.")

        return sensor

    except Exception as e:
        print(f"Error initializing VL53L0X sensor: {e}")
        exit()

# Function to check if sensor is responsive at a given address
def check_sensor(i2c, address):
    try:
        sensor = adafruit_vl53l0x.VL53L0X(i2c, address=address)
        # Perform a read to confirm sensor is responsive
        sensor.range
        return True
    except Exception:
        return False

# Function to apply exponential moving average filter
def apply_ema_filter(ema, new_value, alpha=EMA_ALPHA):
    if ema is None:
        return new_value
    return alpha * new_value + (1 - alpha) * ema

# Function to read distance from sensors and control motors
def read_distance_and_control_motors(i2c, mqtt_client, ser):
    sensors = {}

    for sensor_name, xshut_pin in XSHUT_PINS.items():
        sensors[sensor_name] = initialize_sensor(i2c, xshut_pin, NEW_ADDRESSES[sensor_name])

    try:
        while True:
            distances = {}
            for sensor_name, sensor in sensors.items():
                distance = sensor.range
                distances[sensor_name] = distance

            # Calculate average distance for each sensor
            avg_distance_front = sum(distances.values()) / len(distances)

            # Apply exponential moving average filter to the distance
            avg_distance_front = apply_ema_filter(avg_distance_front, distances['sensor_front'])

            # Control motors based on the averaged distance
            if avg_distance_front < OFFSET:
                # Obstacle detected, stop and avoid obstacle
                stop_motors()
                mqtt_client.publish("robot/status", "Obstacle detected. Stopping and avoiding.")
                turn_left(sensor)  # Adjust turn as needed
            else:
                # No obstacle, continue moving forward
                move_forward()

            # Read and process serial data
            read_and_process_serial_data(ser)

            # Sleep for a short time to control the loop frequency
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Program stopped by user.")
        stop_motors()

# MQTT Integration

MQTT_BROKER = "a988861856734e6381d16cde197811da.s1.eu.hivemq.cloud"
MQTT_PORT = 8883
MQTT_TOPIC = "getdata"
MQTT_USERNAME = "Bhawbhaw5050"
MQTT_PASSWORD = "Bhawbhaw5050"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker.")
    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    message = msg.payload.decode()
    print(f"Received message: {message}")
    
    if message == "move_forward":
        move_forward_with_stop()
    elif message == "turn_left":
        turn_left(sensor)
    elif message == "turn_right":
        turn_right(sensor)
    else:
        print("Unknown command received.")

def initialize_mqtt():
    client = mqtt.Client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.tls_set()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER, MQTT_PORT)

        client.loop_forever()

    except KeyboardInterrupt:
        print("Program stopped by user.")
        client.disconnect()

    except Exception as e:
        print(f"Error in MQTT connection: {e}")
        client.disconnect()

# Serial Communication Integration

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

MSG_LEN = 9
CMD_SET_FREQ_HZ = 0xA1
CMD_GET_FREQ_HZ = 0xB1
CMD_SET_CURRENT_UA = 0xA2
CMD_GET_CURRENT_UA = 0xB2
CMD_SET_MEASURE_DURATION = 0xA3
CMD_GET_MEASURE_DURATION = 0xB3
CMD_START_MEASUREMENT = 0xA4
CMD_GET_IMPEDANCE = 0xB4

NUM_SAMPLES = 50  # Number of samples for better averaging
RETRY_LIMIT = 3  # Number of retries for each sample in case of failure

def open_serial_connection(port, baudrate, timeout):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(1)  # Wait for the device to initialize
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

def send_command(ser, command, value1=0, value2=0):
    message = bytearray([command])
    message.extend(value1.to_bytes(4, 'big'))
    message.extend(value2.to_bytes(4, 'big'))

    ser.reset_output_buffer()
    ser.write(message)
    ser.reset_input_buffer()
    response = ser.read(MSG_LEN)

    if len(response) == MSG_LEN:
        if response[0] == 0xFF:
            print("Error: Error Message from ESP")
            return None, None, None
        else:
            received_command = response[0]
            received_data1 = int.from_bytes(response[1:5], byteorder='big')
            received_data2 = int.from_bytes(response[5:], byteorder='big')
            return received_command, received_data1, received_data2
    else:
        print(f"Error: Unexpected response length. Length = {len(response)}")
        return None, None, None

def read_and_process_serial_data(ser):
    try:
        # Example: read data from serial port
        command, data1, data2 = send_command(ser, CMD_GET_FREQ_HZ)
        if command == CMD_GET_FREQ_HZ:
            print(f"Frequency: {data1} Hz")
        elif command == CMD_GET_CURRENT_UA:
            print(f"Current: {data1} uA")
        elif command == CMD_GET_IMPEDANCE:
            print(f"Impedance: {data1} ohms")
        # Add more cases as needed

    except Exception as e:
        print(f"Error reading serial data: {e}")

# Function to set servo angle
def set_servo_angle(pwm, angle):
    duty_cycle = 2.5 + (angle / 18.0)
    pwm.ChangeDutyCycle(duty_cycle)

# Set GPIO pins for the servos
servo_pin_1 = 17  # GPIO pin for the first servo
servo_pin_2 = 27  # GPIO pin for the second servo
servo_pin_3 = 22 # GPIO pin for the second servo
servo_pin_4 = 4  # GPIO pin for the second servo

# Set PWM parameters
GPIO.setup(servo_pin_1, GPIO.OUT)
GPIO.setup(servo_pin_2, GPIO.OUT)
GPIO.setup(servo_pin_3, GPIO.OUT)
GPIO.setup(servo_pin_4, GPIO.OUT)

#PWM frequency set to 50 Hz for all motors
pwm_1 = GPIO.PWM(servo_pin_1, 50)  
pwm_2 = GPIO.PWM(servo_pin_2, 50) 
pwm_3 = GPIO.PWM(servo_pin_3, 50) 
pwm_4 = GPIO.PWM(servo_pin_4, 50) 

# Start PWM with 0% duty cycle
pwm_1.start(0)
pwm_2.start(0)
pwm_3.start(0)
pwm_4.start(0)

# Initial angles
initial_angle_1 = 170
initial_angle_2 = 0
initial_angle_3 = 170
initial_angle_4 = 0


def main():
    try:
        # Initialize I2C bus for TOF sensors
        i2c = busio.I2C(board.SCL, board.SDA)

        # Initialize serial communication
        ser = open_serial_connection(SERIAL_PORT, BAUD_RATE, timeout=1)
        if ser is None:
            return

        # Initialize MQTT client
        mqtt_client = mqtt.Client()
        mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        mqtt_client.tls_set()
        mqtt_client.on_connect = on_connect
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT)

        # Start the MQTT client loop
        mqtt_client.loop_start()

    # Initialize sensors with new addresses
        sensors = {}
        ema_distances = {}
        for key, xshut_pin in XSHUT_PINS.items():
            if not check_sensor(i2c, NEW_ADDRESSES[key]):
                sensors[key] = initialize_sensor(i2c, xshut_pin, NEW_ADDRESSES[key])
            else:
                sensors[key] = adafruit_vl53l0x.VL53L0X(i2c, address=NEW_ADDRESSES[key])
            print(f"VL53L0X sensor already initialized at address {hex(NEW_ADDRESSES[key])}")
            ema_distances[key] = None
            time.sleep(1)  # Small delay to ensure the address change takes effect

        # Main loop
        while True:
            # Move forward for 2 seconds
            move_forward_with_stop()
            
        
            #move servo motors down
            
            set_servo_angle(pwm_1, 30)  # Adjust servo angles as needed
            set_servo_angle(pwm_2, 150)
            set_servo_angle(pwm_3, 30)
            set_servo_angle(pwm_4, 150)

            # Read average voltage from serial and publish to MQTT
            read_and_process_serial_data(ser)
            mqtt_client.publish("robot/status", "Published average voltage to MQTT.")

            # Move servo motors up
            set_servo_angle(pwm_1, 170)  # Return servos to initial positions
            set_servo_angle(pwm_2, 0)
            set_servo_angle(pwm_3, 170)
            set_servo_angle(pwm_4, 0)

            # Check front TOF sensor distance
            front_distance = sensors['sensor_front'].range
            if front_distance <= 100:
                stop_motors()
                avoid_obstacle(sensors)

            time.sleep(0.1)  # Adjust loop frequency as needed

    except KeyboardInterrupt:
        print("Program stopped by user.")
        stop_motors()

    except Exception as e:
        print(f"Error in main program: {e}")
        stop_motors()

    finally:
        ser.close()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        pwm_1.stop()
        pwm_2.stop()
        pwm_3.stop()
        pwm_4.stop()
        GPIO.cleanup()

# Entry point of the script
if __name__ == "__main__":
    main()
