import time
import board
import busio
import adafruit_vl53l0x
import digitalio

# Define the XSHUT pins for each sensor
XSHUT_PINS = {
    'sensor1': board.D5,  # Example GPIO pin for sensor 1
    'sensor2': board.D6,  # Example GPIO pin for sensor 2
    'sensor3': board.D7   # Example GPIO pin for sensor 3
}

# New addresses for each sensor
NEW_ADDRESSES = {
    'sensor1': 0x30,
    'sensor2': 0x31,
    'sensor3': 0x32
}

def initialize_sensor(i2c, xshut_pin, new_address):
    try:
        xshut = digitalio.DigitalInOut(xshut_pin)
        xshut.direction = digitalio.Direction.OUTPUT
        xshut.value = False  # Keep the sensor in reset state

        time.sleep(0.1)
        xshut.value = True  # Bring the sensor out of reset

        # Try to initialize sensor at the default address
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
        
        if sensor.sensor_address in [0x29, 0x30, 0x31, 0x32]:
            if sensor.sensor_address == new_address:
                print(f"VL53L0X sensor for sensor at address {hex(sensor.sensor_address)} already initialized.")
            else:
                sensor.set_address(new_address)
                print(f"VL53L0X sensor initialized and set to address {hex(new_address)}.")
        else:
            print(f"No VL53L0X sensor found at default address 0x29 for sensor {new_address}. Setting new address...")
            sensor.set_address(new_address)
            print(f"VL53L0X sensor initialized and set to address {hex(new_address)}.")

        return sensor

    except Exception as e:
        print(f"Error initializing VL53L0X sensor: {e}")
        exit()

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Initialize sensors with new addresses
    sensors = {}
    for key, xshut_pin in XSHUT_PINS.items():
        sensors[key] = initialize_sensor(i2c, xshut_pin, NEW_ADDRESSES[key])
        time.sleep(1)  # Small delay to ensure the address change takes effect

    try:
        while True:
            for key, sensor in sensors.items():
                try:
                    distance_mm = sensor.range
                    print(f"Sensor {key} distance: {distance_mm}mm")
                except RuntimeError as e:
                    print(f"Error reading distance from {key}: {e}")

            time.sleep(1.0)  # Adjust refresh rate as needed

    except KeyboardInterrupt:
        print("\nExiting program.")

if __name__ == "__main__":
    main()
