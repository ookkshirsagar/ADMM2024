#!/usr/bin/env python3

import time
import board
import busio
import adafruit_vl53l0x
import digitalio

# Number of samples to average for stability
NUM_SAMPLES = 20

# Offset adjustment based on calibration (if needed)
OFFSET = 20  # Adjust this value based on calibration measurements

# Define the XSHUT pins for each sensor
XSHUT_PINS = {
    'front': board.D5,
    'left': board.D6,
    'right': board.D7
}

# New addresses for each sensor
NEW_ADDRESSES = {
    'front': 0x30,
    'left': 0x31,
    'right': 0x32
}

def initialize_sensor(i2c, xshut_pin, new_address):
    try:
        xshut = digitalio.DigitalInOut(xshut_pin)
        xshut.direction = digitalio.Direction.OUTPUT
        xshut.value = False  # Keep the sensor in reset state

        time.sleep(0.1)
        xshut.value = True  # Bring the sensor out of reset

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
            distance = sensor.range
            distances.append(distance)
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
    sensors = {}
    for key, xshut_pin in XSHUT_PINS.items():
        sensors[key] = initialize_sensor(i2c, xshut_pin, NEW_ADDRESSES[key])
        time.sleep(1)  # Small delay to ensure the address change takes effect

    # Configure each sensor
    for sensor in sensors.values():
        configure_sensor(sensor)
    
    try:
        while True:
            # Clear the screen before printing new readings
            print("\033[H\033[J")
            
            for key, sensor in sensors.items():
                average_distance = get_average_distance(sensor)
                
                if average_distance is not None:
                    adjusted_distance = average_distance - OFFSET
                    print(f"Sensor {key.capitalize()} Averaged Range: {adjusted_distance:.2f} mm")
                else:
                    print(f"Sensor {key.capitalize()}: No valid distance readings.")
            
            # Sleep for a short time before refreshing the readings
            time.sleep(1)  # Adjust the sleep time as needed for your application

    except KeyboardInterrupt:
        print("\nExiting program.")

if __name__ == "__main__":
    main()
