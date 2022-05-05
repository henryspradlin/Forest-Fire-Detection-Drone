import RPi.GPIO as GPIO
from time import sleep

led = 16

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led, GPIO.OUT, initial=GPIO.HIGH)

while True:
    print("on")
    GPIO.output(led, GPIO.HIGH)
    sleep(1)
    print("off")
    GPIO.output(led, GPIO.LOW)
    sleep(1)