from mpu6050 import mpu6050
from time import sleep

# Initialize sensor
sensor = mpu6050(0x68)

# Calibration constants
SF_x = 1 / 10.20
SF_y = 1 / 9.85
SF_z = 1 / 8.5

# Offset calculation for z-axis
z_offset = (SF_x + SF_y) / 2 - SF_z

def get_calibrated_accel_data(sensor):
    raw_data = sensor.get_accel_data()
    calibrated_data = {
        'x': raw_data['x'] * SF_x,
        'y': raw_data['y'] * SF_y,
        'z': raw_data['z'] * SF_z + z_offset
    }
    return calibrated_data

while True:
    # Get raw sensor data
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    temp = sensor.get_temp()

    # Get calibrated acceleration data
    calibrated_accel_data = get_calibrated_accel_data(sensor)

    print("Raw Accelerometer data")
    print(f"x: {accel_data['x']:.2f} y: {accel_data['y']:.2f} z: {accel_data['z']:.2f}")
    
    print("Calibrated Accelerometer data")
    print(f"x: {calibrated_accel_data['x']:.2f} g y: {calibrated_accel_data['y']:.2f} g z: {calibrated_accel_data['z']:.2f} g")

    print("Gyroscope data")
    print(f"x: {gyro_data['x']:.2f} y: {gyro_data['y']:.2f} z: {gyro_data['z']:.2f}")

    print("Temp: " + str(temp) + " C")
    sleep(0.5)
