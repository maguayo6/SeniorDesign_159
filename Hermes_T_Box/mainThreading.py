# Main.py with threading!
# Python program to illustrate the concept 
# of threading 
# importing the threading module 
import threading
import smbus            #import SMBus module of I2C
import RTIMU
from time import sleep          #import
import adafruit_gps
import time
import struct
import board
import busio
import digitalio as dio
from circuitpython_nrf24l01 import RF24
import serial
import os
import can
import obd
import xlwt 
from xlwt import Workbook
from math import degrees

global row, buffer, isFinished
row = 1
buffer = [struct.pack("if",-1,-1.0)]*17
isFinished = [False]*4

class ExceptionThread(threading.Thread):
    """ LogThread is a class dedicated to handling
    exceptions within specific threads to prevent
    the application from crashing.
    """
    def __init__(self,msg, **kwargs):
        super().__init__(**kwargs)
        self.errmsg = msg
        self._real_run = self.start
        self.start = self.execute

    def execute(self):
        try:
            self._real_run()
        except:
            print(self.errmsg)


def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    
    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

  
def loopIMU(num,sheet): 
    """ 
    Function to print IMU Data in a Loop 
    """
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
    imu.setSlerpPower(0.01)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)
    poll_interval = imu.IMUGetPollInterval()
    roll = ""
    pitch = ""
    yaw = ""
    while(num>0):
        if imu.IMURead():
            data = imu.getIMUData()
            #print("Hello")
            fusionPose = data["fusionPose"]
            #global roll, pitch, yaw
            roll = degrees(fusionPose[0])
            pitch = degrees(fusionPose[1])
            yaw = degrees(fusionPose[2])
            global buffer
            buffer[0] = struct.pack("if",1,roll)
            buffer[1] = struct.pack("if",2,pitch)
            buffer[2] = struct.pack("if",3,yaw)
            global row
            sheet.write(row, 0, str(roll))
            sheet.write(row, 1, str(pitch))
            sheet.write(row, 2, str(yaw))
            row+=1
        num-=1
        #print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)     
        sleep(.01)
    global isFinished
    isFinished[0] = True
  
def loopGPS(num,sheet): 
    """ 
    Function to print IMU Data in a Loop 
    """
    uart = serial.Serial("/dev/ttyUSB0", baudrate=9600,timeout=10)

    gps = adafruit_gps.GPS(uart,debug=False)

    gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    gps.send_command(b'PMTK220,10')

    last_print = time.monotonic()
    while num>0:
        # Make sure to call gps.update() every loop iteration and at least twice
        # as fast as data comes from the GPS unit (usually every second).
        # This returns a bool that's true if it parsed new data (you can ignore it
        # though if you don't care and instead look at the has_fix property).
        #print("h")
        #print(gps.readline())
        gps.update()
        # Every second print out current location details if there's a fix.
        current = time.monotonic()
        if current - last_print >= .010:
            num-=1
            last_print = current
            if not gps.has_fix:
                # Try again if we don't have a fix yet.
                #print('Waiting for fix...\n')
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
            gps_lat = gps.latitude
            gps_long = gps.longitude
            global buffer
            buffer[3] = struct.pack("if",4,gps_lat)
            buffer[4] = struct.pack("if",5,gps_long)
            print('Latitude: {0:.6f} degrees'.format(gps_lat))
            print('Longitude: {0:.6f} degrees'.format(gps_long))
            global row
            sheet.write(row, 3, str(gps_lat))
            sheet.write(row, 4, str(gps_long))
            #print('Fix quality: {}'.format(gps.fix_quality))
            # Some attributes beyond latitude, longitude and timestamp are optional
            # and might not be present.  Check if they're None before trying to use!
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
            print("\n")
    global isFinished
    isFinished[1] = True

def loopCAN(num,sheet):
    print("Opening Can Bus")
    print("Check CAN WIRES If Outputs Are Weird!")
    os.system('sudo ip link set can0 up type can bitrate 500000')
    bus = can.Bus(bustype = 'socketcan',channel='can0')
    canbus = bus.recv(0.01)
    global buffer
    while(num>0):
        c = bus.recv(0.01)
        if(c!=None):
            canbus = list(c.data)
            for i,val in enumerate(canbus):
                buffer[5+i] = struct.pack("if",5+i+1,val)
            global row
            sheet.write(row, 5, str(canbus))
            #print(canbus)
        num-=1
    global isFinished
    isFinished[2] = True
    os.system("sudo ip link set can0 down")
    
    
def loopOBD2(num,sheet):
    print("Starting OBD2 Reading.. ")
    connection = obd.OBD("/dev/ttyUSB1")
    carCommands = [obd.commands.ENGINE_LOAD,
    #               obd.commands.COOLANT_TEMP,
    #               obd.commands.FUEL_PRESSURE,
                   obd.commands.RPM,
                   obd.commands.SPEED,
    #               obd.commands.INTAKE_TEMP,
    #               obd.commands.MAF,
                   obd.commands.THROTTLE_POS]
    while(connection.is_connected() and num>0):
        respCommands = []
        for c in carCommands:
            respCommands.append(str(connection.query(c).value))
        global buffer
        for i,val in enumerate(respCommands):
                buffer[13+i] = struct.pack("if",13+i+1,val)
#        buffer[13:17] = respCommands
        global row
        sheet.write(row, 6, str(respCommands[0]))
        sheet.write(row, 7, str(respCommands[1]))
        sheet.write(row, 8, str(respCommands[2]))
        sheet.write(row, 9, str(respCommands[3]))
        num-=1
        time.sleep(0.01)
    global isFinished
    isFinished[3] = True
    
def master(nrf):  # count = 5 will only transmit 5 packets
    """Transmits an incrementing integer every second"""
    # set address of RX node into a TX pipe
    nrf.open_tx_pipe(address)
    # ensures the nRF24L01 is in TX mode
    nrf.listen = False
    
    global isFinished, buffer
    start = time.monotonic()
    success_percentage = 0
    while (False in isFinished):
        result = nrf.send(list(filter(lambda i: i!=struct.pack("if",-1,-1.0), buffer)))
        success_percentage += result.count(True)
    print("Total Sent: ", success_percentage)
    print("Total Time: ", time.monotonic() - start)
    print('Transmission Speed', (time.monotonic() - start)/success_percentage)
#        for ind,val in enumerate(buffer):
#            if(val!=-1):
#                buf = struct.pack('<if', ind,val)
#                # 'i' means a single 4 byte int value.
#                # '<' means little endian byte order. this may be optional
##                print("Sending: {} as struct: {}".format(ind, val))
#                now = time.monotonic() * 1000  # start timer
#                result = nrf.send(buf)
#                if result is None:
#                    pass
##                    print('send() timed out')
#                elif not result:
#                    pass
##                    print('send() failed')
#                else:
##                    print('send() successful')
#                    total+=time.monotonic() * 1000 - now
#                    count+=1
#                # print timer results despite transmission success
#                #print('Transmission took',
#    #                  time.monotonic() * 1000 - now, 'ms')
#    print("Total time :",time.monotonic() - start)
#    print("Total Transmitted: ", count)
#    print("Average Transmission Time for a Data Value: ",str((time.monotonic()- start)/count))
        #time.sleep(0.25)

if __name__ == "__main__":
    
#    #some MPU6050 Registers and their Address
#    PWR_MGMT_1   = 0x6B
#    SMPLRT_DIV   = 0x19
#    CONFIG       = 0x1A
#    GYRO_CONFIG  = 0x1B
#    INT_ENABLE   = 0x38
#    ACCEL_XOUT_H = 0x3B
#    ACCEL_YOUT_H = 0x3D
#    ACCEL_ZOUT_H = 0x3F
#    GYRO_XOUT_H  = 0x43
#    GYRO_YOUT_H  = 0x45
#    GYRO_ZOUT_H  = 0x47
#    
#    bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
#    Device_Address = 0x68   # MPU6050 device address
#
#    MPU_Init()
    
    # Workbook is created 
    wb = Workbook() 
    # add_sheet is used to create sheet. 
    sheet1 = wb.add_sheet('Hermes Telemetry Data',cell_overwrite_ok=True)
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
    
    # addresses needs to be in a buffer protocol object (bytearray)
    address = b'1Node'

    # change these (digital output) pins accordingly
    ce = dio.DigitalInOut(board.D4)
    csn = dio.DigitalInOut(board.D5)

    # using board.SPI() automatically selects the MCU's
    # available SPI pins, board.SCK, board.MOSI, board.MISO
    spi = busio.SPI(board.SCLK_1,board.MOSI_1,board.MISO_1)  # init spi bus object

    # we'll be using the dynamic payload size feature (enabled by default)
    # initialize the nRF24L01 on the spi bus object
    nrf = RF24(spi, csn, ce)
    nrf.data_rate = 2
#    help(nrf)
    
    # creating thread 
#    t1 = threading.Thread(target=loopIMU, args=(100,)) 
#    t2 = threading.Thread(target=loopGPS, args=(10,))
#    t3 = threading.Thread(target=loopCAN, args=(10,))
#    t4 = threading.Thread(target=loopOBD2, args=(10,))
  
#    # starting thread 1 
#    t1.start() 
#    # starting thread 2 
#    t2.start()
#    # starting thread 3 
#    t3.start()
#    # starting thread 4 
#    t4.start()
#  
#    # wait until thread 1 is completely executed 
#    t1.join() 
#    # wait until thread 2 is completely executed 
#    t2.join()
#    # wait until thread 3 is completely executed 
#    t3.join()
#    # wait until thread 4 is completely executed 
#    t4.join()
        
    t1 = ExceptionThread("IMU Sensor Failed!",target=loopIMU, args=(1000,sheet1)) 
    t2 = ExceptionThread("GPS Sensor Failed!",target=loopGPS, args=(1000,sheet1,))
    t3 = ExceptionThread("CAN Bus Failed!",target=loopCAN, args=(1000,sheet1,))
    t4 = ExceptionThread("OBD2 Connection Failed!",target=loopOBD2, args=(1000,sheet1,))
    t5 = ExceptionThread("Transmission Failed!",target=master, args=(nrf,))
    
    t1.execute()
    t2.execute()
    t3.execute()
    t4.execute()
    t5.execute()
    
    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5.join()
    
    wb.save('HermesTelemetryData.xls')
    # both threads completely executed 