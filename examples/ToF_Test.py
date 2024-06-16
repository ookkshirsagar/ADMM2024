#!/home/admm2024/admm/bin/python

import time
import board
import busio
import adafruit_vl53l0x

# Initialize I2C bus and sensor
i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

while True:
    print("Range: {0}mm".format(vl53.range))
    time.sleep(1.0)