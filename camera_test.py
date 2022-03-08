from picamera import PiCamera
from time import sleep
import keyboard

camera = PiCamera()

camera.start_preview()
while True:
    if keyboard.read_key() == "e":
        break

camera.stop_preview()