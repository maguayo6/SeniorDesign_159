from math import degrees
from time import sleep
import RTIMU
#SETTINGS_FILE = "/home/pi/displacedRealityDrone/IMU/RTEllipsoidFit/RTIMULib.ini"
SETTINGS_FILE = "/home/pi/SeniorDesign_159/Hermes_T_Box/IMU/RTEllipsoidFit/RTIMULib.ini"
s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)
if (not imu.IMUInit()):
  print("Failed to initialize IMU")
  exit(1)
else:
  print("Recording IMU Data")
imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)
poll_interval = imu.IMUGetPollInterval()
#print(imu.IMURead())


while(True):
    if imu.IMURead():
        data = imu.getIMUData()
        #print("Hello")
        fusionPose = data["fusionPose"]
        global roll, pitch, yaw
        roll = degrees(fusionPose[0])
        pitch = degrees(fusionPose[1])
        yaw = degrees(fusionPose[2])
        print( str(roll) + "   " + str(pitch) + "   " + str(yaw))
        sleep(0.2)
