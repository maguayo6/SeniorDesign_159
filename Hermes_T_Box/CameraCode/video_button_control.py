#Video recording triggered by pressing a button, includes an indicator LED
'''
import picamera
from time import sleep

with picamera.PiCamera() as camera:
        print("Start Recording.")
        camera.start_recording("/home/pi/Desktop/pythonVideo.h264")
        sleep(5)
        camera.stop_recording()
        print("End Recording")
'''

from picamera import PiCamera
from time import sleep
from gpiozero import Button, LED
import datetime

led = LED(23)
button = Button(17)
camera = PiCamera()

camera.exposure_mode = 'antishake'
button.wait_for_press()
print("Start Recording")
camera.start_recording("/home/pi/Desktop/testVideos/test1.h264")
led.on()
print("Recording")
sleep(30)
button.wait_for_press()
camera.stop_recording()
print("Stop Recording")
led.off()    
