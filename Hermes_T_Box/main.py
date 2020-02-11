from math import degrees
from time import sleep
import RTIMU
import adafruit_gps
import time
import serial
import can
import obd
import time
import csv
import xlwt 
from xlwt import Workbook
import os
from getkey import getkey, keys


print("Starting Heremes Telemtry Box")

# Workbook is created 
wb = Workbook() 
# add_sheet is used to create sheet. 
sheet1 = wb.add_sheet('Hermes Telemetry Data')
sheet1.write(0, 0, "Roll")
sheet1.write(0, 1, "Pitch")
sheet1.write(0, 2, "Yaw")
sheet1.write(0, 3, "GPS Lat")
sheet1.write(0, 4, "GPS Long")
sheet1.write(0, 5, "CAN Bus")
sheet1.write(0, 6, "Engine Load")
sheet1.write(0, 7, "RPM")
sheet1.write(0, 8, "Speed")
sheet1.write(0, 9, "Throttle Pos")

isIMU = True
isGPS = True
isCanBus = True
doneCollecting = False

print("Starting OBD2 Reading.. ")
connection = obd.OBD("/dev/ttyUSB0")
carCommands = [obd.commands.ENGINE_LOAD,
#               obd.commands.COOLANT_TEMP,
#               obd.commands.FUEL_PRESSURE,
               obd.commands.RPM,
               obd.commands.SPEED,
#               obd.commands.INTAKE_TEMP,
#               obd.commands.MAF,
               obd.commands.THROTTLE_POS]

try:
    print("Opening Can Bus")
    print("Check CAN WIRES If Outputs Are Weird!")
    os.system('sudo ip link set can0 up type can bitrate 500000')
    bus = can.Bus(bustype = 'socketcan',channel='can0')
    canbus = bus.recv()
except:
    isCanBus = False
    
try:
    print("Initializing IMU")
    SETTINGS_FILE = "/home/pi/SeniorDesign_159/Hermes_T_Box/IMU/RTEllipsoidFit/RTIMULib.ini"
    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)
    if (not imu.IMUInit()):
      print("Failed to initialize IMU")
      isIMU = False
      exit(1)
    else:
      print("Recording IMU Data")
    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)
    poll_interval = imu.IMUGetPollInterval()
except:
    isIMU = False

try:
    print("Initalizing GPS")
    uart = serial.Serial("/dev/ttyUSB1", baudrate=9600,timeout=10)
    gps = adafruit_gps.GPS(uart,debug=False)
    gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    gps.send_command(b'PMTK220,1000')
    last_print = time.monotonic()
except:
    isGPS = False

roll = "Not Found"
pitch = "Not Found"
yaw = "Not Found"
gps_long = "Not Found"
gps_lat = "Not Found"
canbus = "Not Found"
respCommands = [None,None,None,None]
rowCount = 1
while(not doneCollecting):
    sleep(0.15)
    if(isGPS):
        gps.update()
        # Every second print out current location details if there's a fix.
        current = time.monotonic()
        if current - last_print >= 1.0:
            last_print = current
            if not gps.has_fix:
                # Try again if we don't have a fix yet.
                print('Waiting for fix...')
                continue
            # We have a fix! (gps.has_fix is true)
            # Print out details about the fix like location, date, etc.
            print('=' * 40)  # Print a separator line.
            print('Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}'.format(
                gps.timestamp_utc.tm_mon,   # Grab parts of the time from the
                gps.timestamp_utc.tm_mday,  # struct_time object that holds
                gps.timestamp_utc.tm_year,  # the fix time.  Note you might
                gps.timestamp_utc.tm_hour,  # not get all data like year, day,
                gps.timestamp_utc.tm_min,   # month!
                gps.timestamp_utc.tm_sec))
            gps_long = gps.longitude
            gps_lat = gps.latitude
            print('Latitude: {0:.6f} degrees'.format(gps_lat))
            print('Longitude: {0:.6f} degrees'.format(gps_long))
#            print('Fix quality: {}'.format(gps.fix_quality))
#            # Some attributes beyond latitude, longitude and timestamp are optional
#            # and might not be present.  Check if they're None before trying to use!
#            if gps.satellites is not None:
#                print('# satellites: {}'.format(gps.satellites))
#            if gps.altitude_m is not None:
#                print('Altitude: {} meters'.format(gps.altitude_m))
#            if gps.speed_knots is not None:
#                print('Speed: {} knots'.format(gps.speed_knots))
#            if gps.track_angle_deg is not None:
#                print('Track angle: {} degrees'.format(gps.track_angle_deg))
#            if gps.horizontal_dilution is not None:
#                print('Horizontal dilution: {}'.format(gps.horizontal_dilution))
#            if gps.height_geoid is not None:
#                print('Height geo ID: {} meters'.format(gps.height_geoid))
        else:
            gps_long = "Lost Connection"
            gps_lat = "Lost Connection"
    
    if(isIMU):
        if imu.IMURead():
            data = imu.getIMUData()
            #print("Hello")
            fusionPose = data["fusionPose"]
            #global roll, pitch, yaw
            roll = degrees(fusionPose[0])
            pitch = degrees(fusionPose[1])
            yaw = degrees(fusionPose[2])
        else:
            roll = "Lost Connection"
            pitch = "Lost Connection"
            yaw = "Lost Connection"
    if(isCanBus):
        c = bus.recv(0.2)
        if(c!=None):
            canbus = list(c.data)
    if(connection.is_connected()):
        respCommands = []
        for c in carCommands:
            respCommands.append(str(connection.query(c).value))
        
    print("IMU: ROLL " + str(roll) + " PITCH " + str(pitch) + " YAW " + str(yaw))
    print("CANBus: " + str(canbus))
    print("OBD2 Data"+str(respCommands))
    
    sheet1.write(rowCount, 0, str(roll))
    sheet1.write(rowCount, 1, str(pitch))
    sheet1.write(rowCount, 2, str(yaw))
    sheet1.write(rowCount, 3, str(gps_lat))
    sheet1.write(rowCount, 4, str(gps_long))
    sheet1.write(rowCount, 5, str(canbus))
    sheet1.write(rowCount, 6, str(respCommands[0]))
    sheet1.write(rowCount, 7, str(respCommands[1]))
    sheet1.write(rowCount, 8, str(respCommands[2]))
    sheet1.write(rowCount, 9, str(respCommands[3]))
    rowCount+=1
#    key = getkey()
    if(rowCount>20):
        doneCollecting = True

os.system("sudo ip link set can0 down")
wb.save('HermesTelemetryData.xls') 
    #print(b)
    


