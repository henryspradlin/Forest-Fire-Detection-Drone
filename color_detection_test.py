import cv2
import numpy as np
from picamera import PiCamera
import time

camera = PiCamera()

camera.start_preview()

time.sleep(5)

camera.capture('/home/pi/Documents/Forest-Fire-Detection-Drone/flight_data/pictures/img.jpg')

camera.stop_preview()

img = cv2.imread('/home/pi/Documents/Forest-Fire-Detection-Drone/flight_data/pictures/img.jpg')
rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
median = cv2.medianBlur(rgb,7)

lowcolor =  np.array([140,50,0])
highcolor = np.array([255,170,100])

mask = cv2.inRange(median, lowcolor, highcolor)

kernel = np.ones((3,3), np.uint8)
erode = cv2.erode(mask, kernel, iterations=1)

cv2.imshow('image', img)
cv2.imshow('mask', mask)
cv2.imshow('erode', erode)

time.sleep(5)
cv2.waitKey(0)

count = np.sum(np.nonzero(mask))
print('count =', count)
if count == 0:
    print('Not Red')
else:
    print('There is Red')