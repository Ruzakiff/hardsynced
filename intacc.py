import picamera
import picamera.array
import numpy as np
import smbus
import time
import threading
import RPi.GPIO as GPIO
from PIL import Image, ImageDraw, ImageFont

# Setup for accelerometer
bus = smbus.SMBus(1)
accel_address = 0x68
bus.write_byte_data(accel_address, 0x6B, 0)  # Wake-up command

# GPIO setup for interrupts
interrupt_pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def read_acceleration(addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg+1)
    value = (high << 8) + low
    if value > 32767:
        value -= 65536
    return value / 16384.0

# Accelerometer data storage
accel_data = [0, 0, 0]  # x, y, z
data_lock = threading.Lock()

# Flag to indicate new data is available
new_data_available = False

# Interrupt service routine
def handle_interrupt(channel):
    global new_data_available
    new_data_available = True

# Function to process accelerometer data
def process_accel_data():
    global accel_data, new_data_available
    if new_data_available:
        with data_lock:
            accel_data[0] = read_acceleration(accel_address, 0x3B)
            accel_data[1] = read_acceleration(accel_address, 0x3D)
            accel_data[2] = read_acceleration(accel_address, 0x3F)
        new_data_available = False

# Set up interrupt detection
GPIO.add_event_detect(interrupt_pin, GPIO.RISING, callback=handle_interrupt)

# Function to generate overlay frames
def generate_overlay(camera):
    font = ImageFont.load_default()
    while True:
        with data_lock:
            overlay_text = f"X: {accel_data[0]:.2f}g Y: {accel_data[1]:.2f}g Z: {accel_data[2]:.2f}g"
        img = Image.new('RGB', (camera.resolution[0], camera.resolution[1]))
        draw = ImageDraw.Draw(img)
        draw.text((10, 10), overlay_text, font=font, fill=(255, 255, 255))
        yield np.array(img)

# Main function to handle camera and overlay
def main():
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.framerate = 24
        camera.start_preview()
        overlay_renderer = None
        for overlay_frame in generate_overlay(camera):
            if overlay_renderer:
                overlay_renderer.update(np.getbuffer(overlay_frame))
            else:
                overlay_renderer = camera.add_overlay(np.getbuffer(overlay_frame), layer=3, alpha=64)
            process_accel_data()  # Process data if new data is available
            time.sleep(0.1)  # Sleep briefly to reduce CPU usage

        camera.stop_preview()

# Start the main function
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        GPIO.cleanup()
        GPIO.cleanup()