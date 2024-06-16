#!/home/admm2024/admm/bin/python

import time
import board
import busio
import adafruit_vl53l0x

try:
    print("Initializing VL53L0X sensor...")
    i2c = busio.I2C(board.SCL, board.SDA)
    vl53 = adafruit_vl53l0x.VL53L0X(i2c)
    print("VL53L0X sensor initialized.")

    # Adjust measurement timing budget
    vl53.measurement_timing_budget = 33000  # 33ms

    # Function to get a more stable reading by averaging multiple measurements
    def get_stable_reading(sensor, num_samples=5):
        distances = []
        for _ in range(num_samples):
            distance = sensor.range
            distances.append(distance)
            time.sleep(0.01)  # Small delay between samples to avoid overflow
        return sum(distances) / num_samples

    # Main loop will read the range and print it every second.
    while True:
        try:
            # Get stable reading
            distance = get_stable_reading(vl53)
            print("Range: {:.2f}mm".format(distance))
        except Exception as e:
            print(f"Error reading sensor data: {e}")
        time.sleep(1.0)

except Exception as e:
    print(f"Error initializing VL53L0X sensor: {e}")

finally:
    # Cleanup GPIO resources
    GPIO.cleanup()
