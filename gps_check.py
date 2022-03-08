import serial
from time import sleep
import sys
import pynmea2

ser = serial.Serial("/dev/ttyAMA0", baudrate=9600)
gpgga_info = "$GPGGA,"
GPGGA_buffer = 0
NMEA_buff = 0

print('running')

def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position


# def get_lng(gps_data):
#     good_data = False
#     while (good_data == False):
#         try:
#             newdata = gps_data.decode('utf-8')
#             temp = newdata[0:6]
#             if (temp == '$GPGGA'):
#                 newmsg = pynmea2.parse(newdata)
#                 lng = newmsg.longitude
#                 if lng:
#                     good_data = True
#                     return lng
#         except:
#             print("no workey")
#newdata = ser.readline()
#print(get_lng(newdata))
    
    

# try:
#     while True:
#         received_data = (str)(ser.readline())
#         print(received_data, "/n")

try:
    while True:
        received_data = (str)(ser.readline()) #read NMEA string received
        GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string
        if (GPGGA_data_available>0):
            GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after “$GPGGA,” string
            NMEA_buff = (GPGGA_buffer.split(','))
            nmea_time = []
            nmea_latitude = []
            nmea_longitude = []
            nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
            nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
            nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
            print("NMEA Time: ", nmea_time,'\n')
            lat = (float)(nmea_latitude)
            lat = convert_to_degrees(lat)
            longi = (float)(nmea_longitude)
            longi = convert_to_degrees(longi)
            print ("NMEA Latitude:", lat,"NMEA Longitude:", longi,'\n')
        
except KeyboardInterrupt:
    sys.exit(0)