#Imports
import serial
from time import sleep
import sys
import pynmea2
import string
import codecs
import RPi.GPIO as GPIO
from picamera import PiCamera
import csv
import cv2
import numpy as np
import shutil

#Returns a boolean about possible fire detected
possible_fire = False

#Range with which to test colors
lowcolor = (200,20,0)
highcolor = (255,200,40)

#Lets the program know when it should stop
continue_loop = True

#Temp variable to store current coords of lat and lng, should be list
gps_coords = [-1, -1]

#Tuple to store list of coords on likley pictures
lat_gps_coords = []
lng_gps_coords = []

#Creating ser variable
ser = serial.Serial("/dev/ttyAMA0", baudrate=9600)

#Camera
camera = PiCamera()

#Set up LED
led = 16
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led, GPIO.OUT, initial=GPIO.LOW)


#Recieves the raw gps data and returns the latitude
#If the data is improperly formatted, it returns -1
def get_lat(gps_data):
    try:
        newdata = gps_data.decode('utf-8')
        newmsg = pynmea2.parse(newdata)
        lat = newmsg.latitude
        if lat:
            return lat
    except:
        return -1


#Recieves the raw gps data and returns the longitude
#If the data is improperly formatted, it returns -1
def get_lng(gps_data):
        try:
            newdata = gps_data.decode('utf-8')
            newmsg = pynmea2.parse(newdata)
            lng = newmsg.longitude
            if lng:
                return lng
        except:
            return -1


#Takes picture/turns LEDs on and saves pic as temp
def take_picture():
    GPIO.output(led, GPIO.HIGH)
    camera.capture("/home/pi/Documents/flight_data/temp_img.jpg")
    GPIO.output(led, GPIO.LOW)
    

#Tests the image to see if there is color in the target range
def test_img(img, lowcolor, highcolor):

    thresh = cv2.inRange(img, lowcolor, highcolor)

    count = np.sim(np.nonzero(thresh))
    print('count =', count)
    if count == 0:
        return False
    else:
        return True

#Function to automatically move and name the images if they are possible fires
def move_img(n):
    original = r'/home/pi/Documents/flight_data/pictures/img.jpg'
    target = r'/home/pi/Documents/flight_data/pictures/possible%s.jpg' % n
    shutil.move(original, target)


#Make into another function that returns a list or dict or something with lat and lng
while True:
    newdata = ser.readline()
    lng = get_lng(newdata)
    lat = get_lat(newdata)
    if (lng != -1):
        break
    
gps_coords = [lat, lng]
print('Latitude:', gps_coords[0], 'Longitude:', gps_coords[1])


#2d data in python is annoying
#create 2 lists
#create csv at end of flight loop



#Counts how many possible fire images have been taken
num_possible = 0

#Flight Loop
#1) Turn on LED
#2) Take Picture
#3) Turn off LED
#4) Check for color spectrum
#4.5)   if yes: Record GPS Coords
#5) Wait
while continue_loop:
    
    take_picture()
    img = cv2.imread('/home/pi/Documents/flight_data/pictures/img.jpg')
    
    #loops, calling a new gps data array untill it gets good data, then stores it in the lat and lng variables
    while True:
        newdata = ser.readline()
        lat = get_lat(newdata)
        lng = get_lng(newdata)
        if (lng != -1):
            break    
    print('Latitude:', lat, 'Longitude:', lng)
    
    
    #Tests and saves img and coords if possible fire
    if test_img(img, lowcolor, highcolor):
        lat_gps_coords.append(lat)
        lng_gps_coords.append(lng)
        move_img(num_possible)
        num_possible += 1
         
    
    time.sleep(5)
    continue_loop = False
    
#csv code
# import csv
# 
# coord_count = 0
# 
# with open('colours.csv', 'w', newline='') as csvfile:
#     fieldnames = ['number', 'lat', 'lng']
#     thewriter = csv.DictWriter(csvfile, fieldnames=fieldnames)
#     thewriter.writeheader()
#     for colour in colours:
#         colour_count += 1
#         thewriter.writerow({'number':colour_count, 'colour':colour})
