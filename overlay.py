import cv2
import smbus
import time
import threading
import RPi.GPIO as GPIO
import numpy as np

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
accel_data = {"x": 0, "y": 0, "z": 0}

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
        accel_data["x"] = read_acceleration(accel_address, 0x3B)
        accel_data["y"] = read_acceleration(accel_address, 0x3D)
        accel_data["z"] = read_acceleration(accel_address, 0x3F)
        new_data_available = False

# Set up interrupt detection
GPIO.add_event_detect(interrupt_pin, GPIO.RISING, callback=handle_interrupt)

# Video capture thread
def capture_video():
    cap = cv2.VideoCapture(0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        overlay_text = f"X: {accel_data['x']:.2f}g Y: {accel_data['y']:.2f}g Z: {accel_data['z']:.2f}g"
        cv2.putText(frame, overlay_text, (10, 30), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
        cv2.imshow('Video', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

# Start the video capture thread
video_thread = threading.Thread(target=capture_video)
video_thread.start()

# Main loop modification to handle data processing
try:
    while True:
        process_accel_data()
        time.sleep(0.01)  # Sleep briefly to reduce CPU usage
except KeyboardInterrupt:
    print("Stopping...")
finally:
    GPIO.cleanup()
