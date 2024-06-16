#!/home/admm2024/admm/bin/python

import time
import board
import busio
import adafruit_vl53l0x

try:
    # Initialize I2C bus and VL53L0X sensor
    i2c = busio.I2C(board.SCL, board.SDA)
    vl53 = adafruit_vl53l0x.VL53L0X(i2c)

    # Optionally adjust the measurement timing budget (in nanoseconds)
    # Example: 200000 for 200ms budget (slower but more accurate)
    vl53.measurement_timing_budget = 200000

    print("VL53L0X sensor initialized.")

    # Function to get a stable reading by averaging multiple samples
    def get_stable_reading(sensor):
        distances = []
        for _ in range(5):  # 5 samples for averaging
            distances.append(sensor.range)
            time.sleep(0.01)  # Small delay between samples
        return sum(distances) / len(distances)

    # Main loop to read and print distance every second
    while True:
        try:
            distance = get_stable_reading(vl53)
            print("Range: {:.2f}mm".format(distance))
        except Exception as e:
            print(f"Error reading sensor data: {e}")
        
        time.sleep(1.0)

except Exception as e:
    print(f"Error initializing VL53L0X sensor: {e}")

finally:
    # Clean up resources
    if 'vl53' in globals():
        vl53.deinit()  # Deinitialize the sensor if it was initialized

    print("Program ended.")
