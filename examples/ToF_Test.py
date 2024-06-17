#!/home/admm2024/admm/bin/python

import time
import board
import busio
import adafruit_vl53l0x

# Initialize I2C bus and sensor.
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    vl53 = adafruit_vl53l0x.VL53L0X(i2c)
    print("VL53L0X sensor initialized.")
except Exception as e:
    print(f"Error initializing VL53L0X sensor: {e}")
    exit()

# Activate continuous mode
vl53.continuous_mode()

# Optionally adjust the measurement timing budget for faster response.
vl53.measurement_timing_budget = 33000  # 33ms, adjust as needed

def get_average_distance(sensor, num_samples=5):
    """Get the average distance from the sensor over a number of samples."""
    distances = []
    for _ in range(num_samples):
        try:
            distance = sensor.range
            distances.append(distance)
        except RuntimeError as e:
            print(f"Error reading distance: {e}")
        time.sleep(0.01)  # Small delay between samples to avoid I2C bus overflow

    if distances:
        return sum(distances) / len(distances)
    else:
        return None

# Main loop to read the range and print it continuously.
try:
    while True:
        average_distance = get_average_distance(vl53, num_samples=15)
        if average_distance is not None:
            print(f"Averaged Range: {average_distance:.2f} mm")
        else:
            print("No valid distance readings.")
        time.sleep(0.1)  # Adjust the sleep time as needed for your application

except KeyboardInterrupt:
    print("\nExiting program.")

# Clean up resources
i2c.deinit()
