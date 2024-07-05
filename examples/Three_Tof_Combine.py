#!/home/admm2024/admm/bin/python

import time
import board
import busio
import adafruit_vl53l0x

# Number of samples to average for stability
NUM_SAMPLES = 20

# Offset adjustment based on calibration (if needed)
OFFSET = 20  # Adjust this value based on calibration measurements

# New addresses for each sensor
NEW_ADDRESS_1 = 0x30
NEW_ADDRESS_2 = 0x31
NEW_ADDRESS_3 = 0x32

def initialize_sensor(i2c, new_address):
    try:
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
        sensor.set_address(new_address)
        print(f"VL53L0X sensor initialized and set to address {hex(new_address)}.")
        return sensor
    except Exception as e:
        print(f"Error initializing VL53L0X sensor: {e}")
        exit()

def configure_sensor(sensor):
    try:
        # Set timing budget (lower budget for faster response)
        sensor.measurement_timing_budget = 100000  # 100ms (adjust as needed)
        print("Measurement timing budget set to 100ms.")
    except Exception as e:
        print(f"Error configuring VL53L0X sensor: {e}")
        exit()

def get_average_distance(sensor, num_samples=NUM_SAMPLES):
    distances = []
    for _ in range(num_samples):
        try:
            sensor.start_continuous()  # Start continuous mode
            distance = sensor.range
            distances.append(distance)
            sensor.stop_continuous()  # Stop continuous mode after reading
        except RuntimeError as e:
            print(f"Error reading distance: {e}")
        time.sleep(0.01)  # Small delay between samples to avoid I2C bus overflow

    if distances:
        # Use a simple moving average to smooth the readings
        average_distance = sum(distances) / len(distances)
        return average_distance
    else:
        return None

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Initialize sensors with new addresses
    sensor1 = initialize_sensor(i2c, NEW_ADDRESS_1)
    time.sleep(1)  # Small delay to ensure the address change takes effect
    sensor2 = initialize_sensor(i2c, NEW_ADDRESS_2)
    time.sleep(1)
    sensor3 = initialize_sensor(i2c, NEW_ADDRESS_3)
    time.sleep(1)
    
    # Configure each sensor
    configure_sensor(sensor1)
    configure_sensor(sensor2)
    configure_sensor(sensor3)
    
    try:
        while True:
            average_distance1 = get_average_distance(sensor1)
            average_distance2 = get_average_distance(sensor2)
            average_distance3 = get_average_distance(sensor3)
            
            if average_distance1 is not None:
                adjusted_distance1 = average_distance1 - OFFSET
                print(f"Sensor 1 Averaged Range: {adjusted_distance1:.2f} mm")
            else:
                print("Sensor 1: No valid distance readings.")
                
            if average_distance2 is not None:
                adjusted_distance2 = average_distance2 - OFFSET
                print(f"Sensor 2 Averaged Range: {adjusted_distance2:.2f} mm")
            else:
                print("Sensor 2: No valid distance readings.")
                
            if average_distance3 is not None:
                adjusted_distance3 = average_distance3 - OFFSET
                print(f"Sensor 3 Averaged Range: {adjusted_distance3:.2f} mm")
            else:
                print("Sensor 3: No valid distance readings.")
                
            time.sleep(1)  # Adjust the sleep time as needed for your application

    except KeyboardInterrupt:
        print("\nExiting program.")

if __name__ == "__main__":
    main()
