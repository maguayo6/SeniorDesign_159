import picamera
import time

camera = picamera.PiCamera()
camera.resolution = (640,480)
camera.framerate = 24
camera.start_preview()
camera.annotate_text = "Hello World!"

time.sleep(6)
camera.capture("foo.jpg")