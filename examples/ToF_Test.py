#!/home/admm2024/admm/bin/python

import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO

# Initialize I2C bus and sensor.
try:
    print("Initializing I2C bus...")
    i2c = busio.I2C(board.SCL, board.SDA)
    print("I2C bus initialized.")
    
    print("Initializing VL53L0X sensor...")
    vl53 = adafruit_vl53l0x.VL53L0X(i2c)
    print("VL53L0X sensor initialized.")

    # Optionally adjust the measurement timing budget for a faster response
    vl53.measurement_timing_budget = 33000  # 33ms

    # Main loop will read the range and print it every second.
    while True:
        try:
            # Read the distance
            distance = vl53.range
            print("Range: {:.2f}mm".format(distance))
        except Exception as e:
            print(f"Error reading sensor data: {e}")
        time.sleep(1.0)

except Exception as e:
    print(f"Error during initialization: {e}")

finally:
    # Clean up GPIO resources
    GPIO.cleanup()
