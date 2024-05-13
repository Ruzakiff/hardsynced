import queue
import threading
import time

# Queues for storing data
frame_queue = queue.Queue()
sensor_queue = queue.Queue()

def camera_isr(frame_data):
    timestamp = time.perf_counter()
    frame_queue.put((timestamp, frame_data))

def sensor_isr(sensor_data):
    timestamp = time.perf_counter()
    sensor_queue.put((timestamp, sensor_data))

def data_processor():
    while True:
        if not frame_queue.empty() and not sensor_queue.empty():
            frame_time, frame = frame_queue.get()
            sensor_time, sensor_data = sensor_queue.get()
            if abs(frame_time - sensor_time) < SYNC_THRESHOLD:
                process_synced_data(frame, sensor_data)

def process_synced_data(frame, sensor_data):
    # Processing logic here
    pass

# Setup interrupts
setup_camera_interrupt(camera_isr)
setup_sensor_interrupt(sensor_isr)

# Start processing thread
processor_thread = threading.Thread(target=data_processor)
processor_thread.start()