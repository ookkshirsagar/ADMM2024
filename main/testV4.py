import RPi.GPIO as GPIO
import time
import serial
import struct
import paho.mqtt.client as mqtt
import threading

# Constants for voltage measurement
MSG_LEN = 9
CMD_SET_FREQ_HZ = 0xA1
CMD_GET_FREQ_HZ = 0xB1
CMD_SET_CURRENT_UA = 0xA2
CMD_GET_CURRENT_UA = 0xB2
CMD_SET_MEASURE_DURATION = 0xA3
CMD_GET_MEASURE_DURATION = 0xB3
CMD_START_MEASUREMENT = 0xA4
CMD_GET_IMPEDANCE = 0xB4

NUM_SAMPLES = 50
RETRY_LIMIT = 3

MQTT_BROKER = "a988861856734e6381d16cde197811da.s1.eu.hivemq.cloud"
MQTT_PORT = 8883
MQTT_TOPIC = "getdata"
MQTT_USERNAME = "Bhawbhaw5050"
MQTT_PASSWORD = "Bhawbhaw5050"

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

# Global variables for PWM control
pwm_1 = None
pwm_2 = None
pwm_3 = None
pwm_4 = None

# Global variable to store initial voltage
initial_voltage = None

# Initialize GPIO
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

# Function to initialize PWM
def initialize_pwm():
    global pwm_1, pwm_2, pwm_3, pwm_4
    GPIO.setup(servo_pin_1, GPIO.OUT)
    GPIO.setup(servo_pin_2, GPIO.OUT)
    GPIO.setup(servo_pin_3, GPIO.OUT)
    GPIO.setup(servo_pin_4, GPIO.OUT)

    pwm_1 = GPIO.PWM(servo_pin_1, 50)
    pwm_2 = GPIO.PWM(servo_pin_2, 50)
    pwm_3 = GPIO.PWM(servo_pin_3, 50)
    pwm_4 = GPIO.PWM(servo_pin_4, 50)

    pwm_1.start(0)
    pwm_2.start(0)
    pwm_3.start(0)
    pwm_4.start(0)

# Function to set servo angle
def set_servo_angle(pwm, angle):
    duty_cycle = 2.5 + (angle / 18.0)
    pwm.ChangeDutyCycle(duty_cycle)

# Function to move servos to initial positions
def move_servos_to_initial_positions():
    initial_angle_1 = 140
    initial_angle_2 = 30
    initial_angle_3 = 140
    initial_angle_4 = 30

    set_servo_angle(pwm_1, initial_angle_1)
    set_servo_angle(pwm_2, initial_angle_2)
    set_servo_angle(pwm_3, initial_angle_3)
    set_servo_angle(pwm_4, initial_angle_4)

    print("Servos are UP")
    time.sleep(5)

# Function to move servos down, measure voltage, and publish if significant change
def move_servos_down_and_publish_voltage(ser, mqtt_client):
    global initial_voltage

    set_servo_angle(pwm_1, 170)
    set_servo_angle(pwm_2, 0)
    set_servo_angle(pwm_3, 170)
    set_servo_angle(pwm_4, 0)

    print("Servos are down")
    time.sleep(15)  # Wait 15 seconds for voltage to stabilize

    # Measure final voltage after movement
    final_voltage = measure_average_voltage(ser)

    if final_voltage is not None:
        print(f"Initial Voltage: {initial_voltage}, Final Voltage: {final_voltage}")

        # Compare with initial voltage and check tolerance
        if abs(final_voltage - initial_voltage) > 50:
            publish_to_mqtt(mqtt_client, MQTT_TOPIC, str(final_voltage))
        else:
            print("Voltage change is within tolerance, not publishing.")
    else:
        print("Error: Unable to measure final voltage.")

# Function to measure average voltage
def measure_average_voltage(ser):
    # Collect samples and calculate the average voltage
    voltage_samples = collect_samples(ser, NUM_SAMPLES)

    if voltage_samples:
        filtered_samples = filter_outliers(voltage_samples)
        average_voltage = sum(filtered_samples) / len(filtered_samples)
        return average_voltage
    else:
        return None

# Function to collect impedance samples
def collect_samples(ser, num_samples):
    samples = []
    for _ in range(num_samples):
        command, impedance, _ = admm_start_and_get_measurement(ser)
        if command == CMD_GET_IMPEDANCE:
            samples.append(impedance)
        else:
            print(f"Error: Unexpected command received: {command}")
    return samples

# Function to start measurement and get impedance
def admm_start_and_get_measurement(ser):
    meas_time = admm_get(ser, CMD_START_MEASUREMENT)[1]
    time.sleep(meas_time)
    impedance = admm_get_impedance(ser)
    return impedance

# Function to get impedance
def admm_get_impedance(ser):
    message = bytearray([CMD_GET_IMPEDANCE])
    message.extend((0).to_bytes(4, 'big'))
    message.extend((0).to_bytes(4, 'big'))

    ser.reset_output_buffer()
    ser.write(message)
    ser.reset_input_buffer()
    response = ser.read(MSG_LEN)

    if len(response) == MSG_LEN:
        received_command = response[0]
        received_data1 = struct.unpack('>f', response[1:5])[0]
        received_data2 = struct.unpack('>f', response[5:])[0]
        return received_command, received_data1, received_data2
    else:
        print(f"Error: Unexpected response length. Length = {len(response)}")
        return None, None, None

# Function to publish to MQTT
def publish_to_mqtt(client, topic, message):
    client.publish(topic, message, qos=1)
    print(f"Published message to MQTT topic '{topic}': {message}")

# Function to handle MQTT on connect
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
    else:
        print(f"Failed to connect to MQTT broker. Return code: {rc}")

# Function to handle MQTT on disconnect
def on_disconnect(client, userdata, rc):
    if rc != 0:
        print(f"Unexpected disconnection from MQTT broker. Return code: {rc}")

# Function to open serial connection
def open_serial_connection(port, baudrate, timeout):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(1)
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

# Function to send command to ESP32
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

# Function to get data from ESP32
def admm_get(ser, command):
    return send_command(ser, command)

# Function to clean up GPIO and PWM
def cleanup():
    pwm_1.stop()
    pwm_2.stop()
    pwm_3.stop()
    pwm_4.stop()
    GPIO.cleanup()

# Main function
def main():
    global initial_voltage

    # Initialize PWM and move servos to initial positions
    initialize_pwm()
    move_servos_to_initial_positions()

    # Connect to MQTT broker
    mqtt_client = mqtt.Client()
    mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
    mqtt_client.loop_start()

    # Open serial connection to ESP32
    ser = open_serial_connection('/dev/ttyUSB0', 9600, 1)

    if ser is not None:
        print("Serial connection established")

        # Take initial voltage reading
        initial_voltage = measure_average_voltage(ser)
        if initial_voltage is None:
            print("Error: Unable to measure initial voltage. Exiting.")
            cleanup()
            return

        print(f"Initial Voltage: {initial_voltage}")

        # Main loop
        try:
            while True:
                # Move servos down, measure voltage, and publish if significant change
                move_servos_down_and_publish_voltage(ser, mqtt_client)

                # Move servos up after measurement
                move_servos_up()

                # Wait 5 seconds before the next iteration
                time.sleep(5)

        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Cleaning up...")
            cleanup()
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
            ser.close()
            GPIO.cleanup()
            print("Program exited cleanly.")

    else:
        print("Exiting program due to serial connection error.")

if __name__ == "__main__":
    main()
