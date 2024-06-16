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
# This setting affects the speed and accuracy of distance measurements.
# Lower values (e.g., 33000 microseconds) make measurements faster but less accurate.
vl53.measurement_timing_budget = 33000  # 33ms, adjust as needed

# Main loop to read the range and print it continuously.
try:
    while True:
        try:
            # Read the distance in millimeters
            distance = vl53.range
            print(f"Range: {distance} mm")
        except RuntimeError as e:
            print(f"Error reading distance: {e}")
        time.sleep(0.1)  # Adjust the sleep time as needed for your application

except KeyboardInterrupt:
    print("\nExiting program.")

# Clean up resources
i2c.deinit()
