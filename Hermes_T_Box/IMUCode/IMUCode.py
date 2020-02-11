#!/usr/bin/python3
from math import degrees
from time import sleep
import RTIMU

class IMUData:
    def __init__(self):
        #SETTINGS_FILE = "/home/pi/displacedRealityDrone/IMU/RTEllipsoidFit/RTIMULib.ini"
        SETTINGS_FILE = "/home/pi/SeniorDesign_159/Hermes_T_Box/IMU/RTEllipsoidFit/RTIMULib.ini"
        s = RTIMU.Settings(SETTINGS_FILE)
        self.imu = RTIMU.RTIMU(s)
        if (not self.imu.IMUInit()):
          print("Failed to initialize IMU")
          exit(1)
        else:
          print("Recodring IMU Data")
        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)
        poll_interval = self.imu.IMUGetPollInterval()
        print(self.imu.IMURead())


    def getData(self):
        
        if self.imu.IMURead():
            data = self.imu.getIMUData()
            print("Hello")
            fusionPose = data["fusionPose"]
            global roll, pitch, yaw
            roll = degrees(fusionPose[0])
            pitch = degrees(fusionPose[1])
            yaw = degrees(fusionPose[2])
            return str(roll) + "   " + str(pitch) + "   " + str(yaw)
            #sleep(0.2)