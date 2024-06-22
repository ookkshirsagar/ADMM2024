import RPi.GPIO as GPIO
import time

# Motor Driver 1 Pins (Left Motors)
left_front_in1 = 23
left_front_in2 = 24
left_front_en = 18

# Motor Driver 2 Pins (Right Motors)
right_front_in1 = 16
right_front_in2 = 20
right_front_en = 13

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Set up Left Front Motor (Motor Driver 1)
    GPIO.setup(left_front_in1, GPIO.OUT)
    GPIO.setup(left_front_in2, GPIO.OUT)
    GPIO.setup(left_front_en, GPIO.OUT)

    # Set up Right Front Motor (Motor Driver 2)
    GPIO.setup(right_front_in1, GPIO.OUT)
    GPIO.setup(right_front_in2, GPIO.OUT)
    GPIO.setup(right_front_en, GPIO.OUT)

    # Enable the motors
    GPIO.output(left_front_en, GPIO.HIGH)
    GPIO.output(right_front_en, GPIO.HIGH)

    print("GPIO setup complete and motors enabled.")

def test_motor(motor_in1, motor_in2):
    # Set motor to move forward
    GPIO.output(motor_in1, GPIO.HIGH)
    GPIO.output(motor_in2, GPIO.LOW)
    time.sleep(2)
    # Stop motor
    GPIO.output(motor_in1, GPIO.LOW)
    GPIO.output(motor_in2, GPIO.LOW)

def main():
    setup()
    try:
        print("Testing Left Front Motor...")
        test_motor(left_front_in1, left_front_in2)
        time.sleep(1)

        print("Testing Right Front Motor...")
        test_motor(right_front_in1, right_front_in2)
        time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting program.")

    finally:
        print("Cleaning up GPIO...")
        GPIO.cleanup()

if __name__ == "__main__":
    main()
