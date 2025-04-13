import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
RELAY_MOTOR = 23
GPIO.setup(RELAY_MOTOR, GPIO.OUT)

print("Turning relay ON...")
GPIO.output(RELAY_MOTOR, GPIO.HIGH)
time.sleep(3)

print("Turning relay OFF...")
GPIO.output(RELAY_MOTOR, GPIO.LOW)

GPIO.cleanup()
