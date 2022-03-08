import cv2
import numpy as np

img = cv2.imread('/home/pi/Documents/flight_data/pictures/img.jpg')

lowcolor =  (0,0,255)
highcolor = (128,128,225)

thresh = cv2.inRange(img, lowcolor, highcolor)

count = np.sim(np.nonzero(thresh))
print('count =', count)
if count == 0:
    print('Not Red')
else:
    print('There is Red')