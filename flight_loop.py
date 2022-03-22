#Imports
import serial
import time
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
lowcolor =  np.array([140,50,0])
highcolor = np.array([255,170,128])

#Lets the program know when it should stop
continue_loop = True

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
    camera.capture("/home/pi/Documents/Forest-Fire-Detection-Drone/flight_data/pictures/temp_img.jpg")
    GPIO.output(led, GPIO.LOW)
    

#Tests the image to see if there is color in the target range
def test_img(img, lowcolor, highcolor):
    
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    median = cv2.medianBlur(rgb,7)
    thresh = cv2.inRange(median, lowcolor, highcolor)

    count = np.sum(np.nonzero(thresh))
    if count == 0:
        print('Not Red')
        return False
    else:
        print('There is Red')
        return True

#Function to automatically move and name the images if they are possible fires
def move_img(n):
    original = r'/home/pi/Documents/Forest-Fire-Detection-Drone/flight_data/pictures/temp_img.jpg'
    target = r'/home/pi/Documents/Forest-Fire-Detection-Drone/flight_data/pictures/possible%s.jpg' % n
    shutil.move(original, target)


#Make into another function that returns a list or dict or something with lat and lng
while True:
    newdata = ser.readline()
    lng = get_lng(newdata)
    lat = get_lat(newdata)
    if (lng != -1):
        break
    

print('Latitude:', lat, 'Longitude:', lng)




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
    img = cv2.imread('/home/pi/Documents/Forest-Fire-Detection-Drone/flight_data/pictures/temp_img.jpg')
    
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
    if num_possible >= 2:
        continue_loop = False
        
    
#csv code

with open('data.csv', 'w', newline='') as csvfile:
    fieldnames = ['number', 'lat', 'lng']
    thewriter = csv.DictWriter(csvfile, fieldnames=fieldnames)
    thewriter.writeheader()
    coord_count = 0
    for x in range(num_possible):
        thewriter.writerow({'number':coord_count, 'lat':lat_gps_coords[coord_count], 'lng':lng_gps_coords[coord_count]})
        coord_count += 1
