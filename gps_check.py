import serial
from time import sleep
import sys
import pynmea2

ser = serial.Serial("/dev/ttyAMA0", baudrate=9600)
gpgga_info = "$GPGGA,"
GPGGA_buffer = 0
NMEA_buff = 0

print('running')

def get_lat(gps_data):
    try:
        newdata = gps_data.decode('utf-8')
        newmsg = pynmea2.parse(newdata)
        lat = newmsg.latitude
        if lat:
            return lat
    except:
        return -1

def get_lng(gps_data):
        try:
            newdata = gps_data.decode('utf-8')
            newmsg = pynmea2.parse(newdata)
            lng = newmsg.longitude
            print(newmsg)
            if lng:
                return lng
        except:
            return -1
        
while True:
    newdata = ser.readline()
    lng = get_lng(newdata)
    lat = get_lat(newdata)
    if (lng != -1):
        break
    
print('Latitude:', lat, 'Longitude:', lng)