from mpu6050 import mpu6050
from time import sleep

# Initialize sensor
sensor = mpu6050(0x68)

# Calibration constants for accelerometer (if needed)
SF_x = 1 / 10.20
SF_y = 1 / 9.85
SF_z = 1 / 8.25

# Gyroscope calibration variables
calibration_samples = 100  # Increased for better averaging

def get_calibrated_accel_data(sensor):
    raw_data = sensor.get_accel_data()
    calibrated_data = {
        'x': raw_data['x'] * SF_x,
        'y': raw_data['y'] * SF_y,
        'z': raw_data['z'] * SF_z
    }
    return calibrated_data

def calibrate_gyroscope(sensor):
    print("Calibrating gyroscope... Keep the sensor still!")

    sum_x = 0
    sum_y = 0
    sum_z = 0

    for _ in range(calibration_samples):
        gyro_data = sensor.get_gyro_data()
        sum_x += gyro_data['x']
        sum_y += gyro_data['y']
        sum_z += gyro_data['z']
        sleep(0.01)  # Adjust as needed

    avg_x = sum_x / calibration_samples
    avg_y = sum_y / calibration_samples
    avg_z = sum_z / calibration_samples

    print(f"Averaged Gyroscope offsets:")
    print(f"X: {avg_x:.2f}")
    print(f"Y: {avg_y:.2f}")
    print(f"Z: {avg_z:.2f}")

    return avg_x, avg_y, avg_z

def get_calibrated_gyro_data(sensor, avg_x, avg_y, avg_z):
    raw_data = sensor.get_gyro_data()
    calibrated_data = {
        'x': raw_data['x'] - avg_x,
        'y': raw_data['y'] - avg_y,
        'z': raw_data['z'] - avg_z
    }
    return calibrated_data

# Perform gyroscope calibration
avg_gyro_x, avg_gyro_y, avg_gyro_z = calibrate_gyroscope(sensor)

while True:
    # Get raw sensor data
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    temp = sensor.get_temp()

    # Get calibrated acceleration data (if needed)
    calibrated_accel_data = get_calibrated_accel_data(sensor)

    # Get calibrated gyroscope data
    calibrated_gyro_data = get_calibrated_gyro_data(sensor, avg_gyro_x, avg_gyro_y, avg_gyro_z)

    print("Raw Accelerometer data")
    print(f"x: {accel_data['x']:.2f} y: {accel_data['y']:.2f} z: {accel_data['z']:.2f}")
    
    if 'calibrated_accel_data' in locals():
        print("Calibrated Accelerometer data")
        print(f"x: {calibrated_accel_data['x']:.2f} g y: {calibrated_accel_data['y']:.2f} g z: {calibrated_accel_data['z']:.2f} g")

    print("Calibrated Gyroscope data")
    print(f"x: {calibrated_gyro_data['x']:.2f} y: {calibrated_gyro_data['y']:.2f} z: {calibrated_gyro_data['z']:.2f}")

    print("Temp: " + str(temp) + " C")
    sleep(0.5)
