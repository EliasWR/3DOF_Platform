"""
from pygrabber.dshow_graph import FilterGraph

def get_available_cameras() :

    devices = FilterGraph().get_input_devices()

    available_cameras = {}

    for device_index, device_name in enumerate(devices):
        available_cameras[device_index] = device_name

    return available_cameras

print(get_available_cameras())
"""
import os
import cv2

def find_camera_port():
    device_list = os.listdir('/dev')
    video_devices = [device for device in device_list if device.startswith('video')]

    for video_device in video_devices:
        camera_port = f'/dev/{video_device}'
        cap = cv2.VideoCapture(camera_port)
        if cap is not None and cap.isOpened():
            cap.release()
            return camera_port

    return None

camera_port = find_camera_port()
if camera_port is not None:
    print(f"Camera found at port: {camera_port}")
else:
    print("Camera not found.")