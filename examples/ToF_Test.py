# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import busio
import adafruit_vl53l0x

# Initialize I2C bus and sensor.
i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

# Optionally adjust the measurement timing budget to change speed and accuracy.
# For example, a slower but more accurate timing budget of 200ms:
vl53.measurement_timing_budget = 200000  # 200ms

# Function to get a more stable reading by averaging multiple measurements
def get_stable_reading(sensor, num_samples=10):
    distances = []
    for _ in range(num_samples):
        distance = sensor.range
        distances.append(distance)
        time.sleep(0.05)  # Small delay between samples
    return sum(distances) / num_samples

# Main loop will read the range and print it every second.
while True:
    # Get stable reading
    distance = get_stable_reading(vl53)
    print("Range: {:.2f}mm".format(distance))
    time.sleep(1.0)
