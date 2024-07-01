import time
from mpu6050 import mpu6050

# MPU6050 sensor address
sensor_address = 0x68  # Check your MPU6050 address
sensor = mpu6050(sensor_address)

# Gyro calibration parameters
calibration_samples = 100  # Number of samples for gyro calibration

# Function for gyro calibration
def calibrate_gyro(sensor):
    print("Calibrating gyro... Keep the sensor still!")
    gyro_offsets = [0, 0, 0]

    for _ in range(calibration_samples):
        gyro_data = sensor.get_gyro_data()
        gyro_offsets[0] += gyro_data['x']
        gyro_offsets[1] += gyro_data['y']
        gyro_offsets[2] += gyro_data['z']
        time.sleep(0.01)

    gyro_offsets = [offset / calibration_samples for offset in gyro_offsets]
    print(f"Gyro offsets: {gyro_offsets}")

    return gyro_offsets

# Function to tare (zero) MPU6050 initial angle
def tare_mpu6050(sensor):
    print("Taring MPU6050... Keeping sensor still!")
    initial_angle = 0.0

    for _ in range(calibration_samples):
        gyro_data = sensor.get_gyro_data()
        initial_angle += gyro_data['z']
        time.sleep(0.01)

    initial_angle /= calibration_samples
    print(f"Initial angle: {initial_angle:.2f} deg")

    return initial_angle

# Function to calculate current angle using gyro data
def calculate_current_angle(sensor, gyro_offsets, initial_angle):
    dt = 0.01  # Sample time
    gyro_integrated_angle = initial_angle

    try:
        while True:
            gyro_data = sensor.get_gyro_data()
            gyro_x = gyro_data['x'] - gyro_offsets[0]
            gyro_y = gyro_data['y'] - gyro_offsets[1]
            gyro_z = gyro_data['z'] - gyro_offsets[2]

            angle_z = gyro_z * dt
            gyro_integrated_angle += angle_z

            # Print current angle
            print(f"Current Angle: {gyro_integrated_angle:.2f} deg")

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nStopping angle calculation.")

# Main program
try:
    # Calibrate gyro and get offsets
    gyro_offsets = calibrate_gyro(sensor)

    # Tare MPU6050 and get initial angle
    initial_angle = tare_mpu6050(sensor)

    # Calculate and print current angle continuously
    calculate_current_angle(sensor, gyro_offsets, initial_angle)

except KeyboardInterrupt:
    print("\nExiting program.")
