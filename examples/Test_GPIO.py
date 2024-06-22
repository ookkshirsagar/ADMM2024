import RPi.GPIO as GPIO
import time

# Test GPIO Pin
test_pin = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(test_pin, GPIO.OUT)

try:
    while True:
        GPIO.output(test_pin, GPIO.HIGH)
        print("GPIO High")
        time.sleep(1)
        GPIO.output(test_pin, GPIO.LOW)
        print("GPIO Low")
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
