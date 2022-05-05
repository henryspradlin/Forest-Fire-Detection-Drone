#! /usr/bin/python3

#Things Left To Do
#Connect with remote switch
#Run on boot

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

#Creating ser variable
ser = serial.Serial("/dev/ttyAMA0", baudrate=9600)

#Camera
camera = PiCamera()

#Set up LED
camera_led = 16
status_led = 18
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(camera_led, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(status_led, GPIO.OUT, initial=GPIO.LOW)

#Lets the program know when it should stop
continue_loop = True

#Returns a boolean about possible fire detected
possible_fire = False

#Tuple to store list of coords on likley pictures
lat_gps_coords = []
lng_gps_coords = []

#Range with which to test colors
lowcolor =  np.array([140,50,0])
highcolor = np.array([255,170,100])

#Erosion matrix size
erode_kernel = np.ones((7,7), np.uint8)

#Time in between pictures
loop_time = 5


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
def take_picture(camera_led):
    GPIO.output(camera_led, GPIO.LOW)
    camera.capture("/home/pi/Documents/Forest-Fire-Detection-Drone/flight_data/pictures/temp_img.jpg")
    GPIO.output(camera_led, GPIO.HIGH)


#Tests the image to see if there is color in the target range
def test_img(img, lowcolor, highcolor, erode_kernel):  
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    thresh = cv2.inRange(rgb, lowcolor, highcolor)
    eroded_img = cv2.erode(thresh, erode_kernel, iterations=1)

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
    target = r'/media/pi/ESD-USB/flight_data/pictures/possible%s.jpg' % n
    shutil.move(original, target)

#Start of the program

#Turns LED on as indicator light untill the loop starts
GPIO.output(status_led, GPIO.HIGH)
time.sleep(5)
GPIO.output(camera_led, GPIO.HIGH)




#Counts how many possible fire images have been taken
num_possible = 0
looped = 0

#Flight Loop
#1) Turn on LED
#2) Take Picture
#3) Turn off LED
#4) Check for color spectrum
#4.5)   if yes: Record GPS Coords
#5) Wait
while continue_loop:
    looped += 1
    take_picture(camera_led)
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
    if test_img(img, lowcolor, highcolor, erode_kernel):
        lat_gps_coords.append(lat)
        lng_gps_coords.append(lng)
        move_img(num_possible)
        num_possible += 1
         
    #Ends loop when 2 possible images have been taken
    time.sleep(loop_time)
    if looped >= 10:
        continue_loop = False
    else:
        continue_loop = True
        

GPIO.setup(status_led, GPIO.OUT, initial=GPIO.LOW)

#csv code
with open('/media/pi/ESD-USB/flight_data/flight_data.csv', 'w', newline='') as csvfile:
    fieldnames = ['index', 'lat', 'lng']
    thewriter = csv.DictWriter(csvfile, fieldnames=fieldnames)
    thewriter.writeheader()
    coord_count = 0
    for x in range(num_possible):
        thewriter.writerow({'index':coord_count, 'lat':lat_gps_coords[coord_count], 'lng':lng_gps_coords[coord_count]})
        coord_count += 1
        
GPIO.output(status_led, GPIO.LOW)
GPIO.output(camera_led, GPIO.LOW)