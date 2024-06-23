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

    # Set up PWM for motor speed control
    global pwm_left_front, pwm_right_front
    pwm_left_front = GPIO.PWM(left_front_en, 1000)
    pwm_right_front = GPIO.PWM(right_front_en, 1000)

    # Start PWM with a duty cycle of 0 (motors off)
    pwm_left_front.start(0)
    pwm_right_front.start(0)

    print("GPIO setup complete.")

def test_motor(motor_in1, motor_in2, pwm_motor):
    GPIO.output(motor_in1, GPIO.HIGH)
    GPIO.output(motor_in2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(100)
    time.sleep(2)
    GPIO.output(motor_in1, GPIO.LOW)
    GPIO.output(motor_in2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(0)

def main():
    setup()
    try:
        print("Testing Left Front Motor...")
        test_motor(left_front_in1, left_front_in2, pwm_left_front)
        time.sleep(1)

        print("Testing Right Front Motor...")
        test_motor(right_front_in1, right_front_in2, pwm_right_front)
        time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting program.")

    finally:
        print("Cleaning up GPIO...")
        pwm_left_front.stop()
        pwm_right_front.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
