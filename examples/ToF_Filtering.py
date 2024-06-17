#!/home/admm2024/admm/bin/python

import time
import board
import busio
import adafruit_vl53l0x

# EMA alpha value (smoothing factor)
EMA_ALPHA = 0.1  # 0 < EMA_ALPHA < 1, higher values = less smoothing

def initialize_sensor():
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
        print("VL53L0X sensor initialized.")
        return sensor
    except Exception as e:
        print(f"Error initializing VL53L0X sensor: {e}")
        exit()

def configure_sensor(sensor):
    try:
        # Set timing budget (higher values are more accurate but slower)
        sensor.measurement_timing_budget = 200000  # 200ms
        print("Measurement timing budget set to 200ms.")
        
        # Start continuous mode
        sensor.start_continuous()
        print("Continuous mode started.")
    except Exception as e:
        print(f"Error configuring VL53L0X sensor: {e}")
        exit()

def get_distance(sensor):
    try:
        distance = sensor.range
        return distance
    except RuntimeError as e:
        print(f"Error reading distance: {e}")
        return None

def main():
    sensor = initialize_sensor()
    configure_sensor(sensor)

    # Initialize EMA with the first reading
    initial_distance = get_distance(sensor)
    if initial_distance is None:
        print("Failed to get initial distance reading.")
        exit()
    ema_distance = initial_distance

    try:
        while True:
            distance = get_distance(sensor)
            if distance is not None:
                # Apply EMA filtering
                ema_distance = EMA_ALPHA * distance + (1 - EMA_ALPHA) * ema_distance
                print(f"Filtered Range: {ema_distance:.2f} mm")
            else:
                print("No valid distance reading.")
            time.sleep(0.1)  # Adjust the sleep time as needed for your application

    except KeyboardInterrupt:
        print("\nExiting program.")

    finally:
        sensor.stop_continuous()
        print("Continuous mode stopped.")

if __name__ == "__main__":
    main()
