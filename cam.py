from picamera import PiCamera
from time import sleep
#can be faster and more efficient. 
# This is because picamera is specifically designed to work with the Raspberry Pi Camera hardware and can take advantage of hardware acceleration.
camera = PiCamera()
camera.resolution = (1024, 768)
camera.start_preview()
# Camera warm-up time
sleep(2)
camera.start_recording('/home/user/video.h264')
sleep(10)
camera.stop_recording()
camera.stop_preview()