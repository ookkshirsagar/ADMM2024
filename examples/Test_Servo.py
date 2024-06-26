from gpiozero import Servo
from time import sleep
from signal import pause

# Define the GPIO pin connected to the servo signal wire
servo_pin = 17

# Initialize the servo object
servo = Servo(servo_pin)

# Function to set the servo angle
def set_servo_angle(angle):
    # Servo position ranges from -1 (0 degrees) to +1 (180 degrees)
    servo.value = (angle / 90.0) - 1.0

try:
    print("Sweeping servo...")
    while True:
        # Sweep from 0 to 180 degrees
        for angle in range(0, 181, 1):
            set_servo_angle(angle)
            sleep(0.01)
        # Sweep back from 180 to 0 degrees
        for angle in range(180, -1, -1):
            set_servo_angle(angle)
            sleep(0.01)

except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    servo.detach()
    print("Servo motor detached.")
