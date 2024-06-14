#!/home/admm2024/admm/bin/python

import time
import board
import busio
import adafruit_vl53l0x

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the ToF sensor
sensor = adafruit_vl53l0x.VL53L0X(i2c)

while True:
    # Read the distance in millimeters
    distance = sensor.range
    print("Distance: {} mm".format(distance))
    time.sleep(1)
