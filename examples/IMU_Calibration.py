import time
from mpu6050 import mpu6050

# MPU6050 sensor address
sensor_address = 0x68
sensor = mpu6050(sensor_address)

# Number of samples to collect for calibration
calibration_samples = 1000

def calibrate_mpu6050(sensor, calibration_samples):
    print("Calibrating MPU6050... Keep the sensor still!")
    
    gyro_offsets = [0.0, 0.0, 0.0]
    accel_offsets = [0.0, 0.0, 0.0]
    
    for i in range(calibration_samples):
        gyro_data = sensor.get_gyro_data()
        accel_data = sensor.get_accel_data()
        
        gyro_offsets[0] += gyro_data['x']
        gyro_offsets[1] += gyro_data['y']
        gyro_offsets[2] += gyro_data['z']
        
        accel_offsets[0] += accel_data['x']
        accel_offsets[1] += accel_data['y']
        accel_offsets[2] += accel_data['z']
        
        time.sleep(0.01)  # Adjust sample rate as necessary
    
    gyro_offsets = [offset / calibration_samples for offset in gyro_offsets]
    accel_offsets = [offset / calibration_samples for offset in accel_offsets]
    
    print(f"Gyro offsets: {gyro_offsets}")
    print(f"Accel offsets: {accel_offsets}")
    
    return gyro_offsets, accel_offsets

def get_corrected_gyro_data(sensor, gyro_offsets):
    gyro_data = sensor.get_gyro_data()
    corrected_gyro = {
        'x': gyro_data['x'] - gyro_offsets[0],
        'y': gyro_data['y'] - gyro_offsets[1],
        'z': gyro_data['z'] - gyro_offsets[2]
    }
    return corrected_gyro

def get_corrected_accel_data(sensor, accel_offsets):
    accel_data = sensor.get_accel_data()
    corrected_accel = {
        'x': accel_data['x'] - accel_offsets[0],
        'y': accel_data['y'] - accel_offsets[1],
        'z': accel_data['z'] - accel_offsets[2]
    }
    return corrected_accel

# Main function to run calibration
if __name__ == "__main__":
    try:
        gyro_offsets, accel_offsets = calibrate_mpu6050(sensor, calibration_samples)
        
        print("\nCalibration complete. Now reading corrected values:")
        while True:
            corrected_gyro = get_corrected_gyro_data(sensor, gyro_offsets)
            corrected_accel = get_corrected_accel_data(sensor, accel_offsets)
            
            print(f"Corrected Gyro: {corrected_gyro}")
            print(f"Corrected Accel: {corrected_accel}")
            
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\nExiting program.")
