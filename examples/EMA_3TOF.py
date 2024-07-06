#!/home/admm2024/admm/bin/python

import time
import board
import busio
import RPi.GPIO as GPIO
import adafruit_vl53l0x

# Number of samples to average for stability
NUM_SAMPLES = 20

# Offset adjustment based on calibration (if needed)
OFFSET = 20  # Adjust this value based on calibration measurements

# GPIO pins for XSHUT control
XSHUT_PINS = {
    'sensor1': 5,
    'sensor2': 6,
    'sensor3': 7
}

# New I2C addresses for each sensor
NEW_ADDRESSES = {
    'sensor1': 0x30,
    'sensor2': 0x31,
    'sensor3': 0x32
}

# Exponential Moving Average (EMA) alpha
EMA_ALPHA = 0.1

def initialize_sensor(xshut_pin, new_address):
    try:
        GPIO.output(xshut_pin, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(xshut_pin, GPIO.HIGH)
        time.sleep(0.1)

        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
        sensor.set_address(new_address)
        print(f"VL53L0X sensor initialized at address {hex(new_address)}.")
        return sensor
    except Exception as e:
        print(f"Error initializing VL53L0X sensor: {e}")
        exit()

def configure_sensor(sensor):
    try:
        sensor.measurement_timing_budget = 100000  # 100ms (adjust as needed)
        print("Measurement timing budget set to 100ms.")
    except Exception as e:
        print(f"Error configuring VL53L0X sensor: {e}")
        exit()

def get_distance(sensor):
    try:
        sensor.start_continuous()
        distance = sensor.range
        sensor.stop_continuous()
        return distance
    except RuntimeError as e:
        print(f"Error reading distance: {e}")
        return None

def apply_ema_filter(ema, new_value, alpha=EMA_ALPHA):
    if ema is None:
        return new_value
    return alpha * new_value + (1 - alpha) * ema

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Set up XSHUT pins
    for pin in XSHUT_PINS.values():
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    # Initialize sensors
    sensors = {}
    for name, pin in XSHUT_PINS.items():
        sensors[name] = initialize_sensor(pin, NEW_ADDRESSES[name])
        configure_sensor(sensors[name])
        time.sleep(1)

    ema_distances = {name: None for name in sensors.keys()}

    try:
        while True:
            for name, sensor in sensors.items():
                distance = get_distance(sensor)
                if distance is not None:
                    distance -= OFFSET  # Apply offset
                    ema_distances[name] = apply_ema_filter(ema_distances[name], distance)
                    print(f"{name}: {ema_distances[name]:.2f} mm")
                else:
                    print(f"{name}: No valid distance readings.")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
