import pyrealsense2 as rs
import numpy as np
import argparse
from os import makedirs
from os.path import exists, join
import shutil
import json
from enum import IntEnum
import time

import board
import digitalio

import open3d as o3d
# from open3d.cuda.pybind.utility import Vector3dVector

cam303 = digitalio.DigitalInOut(board.C1)
cam303.direction = digitalio.Direction.OUTPUT
cam608 = digitalio.DigitalInOut(board.C3)
cam608.direction = digitalio.Direction.OUTPUT
cam337 = digitalio.DigitalInOut(board.C2)
cam337.direction = digitalio.Direction.OUTPUT
cam630 = digitalio.DigitalInOut(board.C0)
cam630.direction = digitalio.Direction.OUTPUT

cam_list = []
cam_list.append(cam303)
cam_list.append(cam608)
cam_list.append(cam337)
cam_list.append(cam630)

photos = 1

# RUN comand example
# python realsense_recorder.py --record_imgs --output_folder=/home/adam/SynappsTech/scans/mult1

connect_device = []
context = rs.context()
for d in context.devices:
    if d.get_info(rs.camera_info.name).lower() != 'platform camera':
        serial = d.get_info(rs.camera_info.serial_number)
        connect_device.append(serial)
        d.hardware_reset()
        print("Device {0} hardware reset done.".format(serial))
