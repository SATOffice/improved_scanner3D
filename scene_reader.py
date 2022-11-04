import json
import sys
import traceback

import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from enum import IntEnum
import time
import board
import digitalio
import marker_detector as md
import logging as log
import scene as sc


log.basicConfig(level=log.INFO)
log = log.getLogger("scene_reader")


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


# L515 reading data
class SceneReader:
    def __init__(self, depth_scale=4000):
        # THE ORDER OF THE SERIALS
        # DETERMINATE THE REAL READ ORDER!!!
        self.gpio_config = {
            'f0264630': board.C0,
            'f0264608': board.C2,
            #'f0270337': board.C1,
            'f0233303': board.C4,
            'f0265553': board.C3
        }
        # load intrinsic configs for sensors
        with open('sensors_data.json') as fd:
            self.sensors_config = json.load(fd)

        # Create a pipeline
        self.depth_scale = depth_scale
        self.device_serials = []
        self.pipeline = {}
        self.config = {}
        self.profile = {}
        self.depth_sensors = {}
        self.depth_scale = {}
        # cameras intrinsic
        self.tt_1 = {}
        self.intr_1 = {}
        self.pinhole_camera_params = {}
        # FT232H control objects
        self.cameras_gpio = {}
        self.clipping_distance = {}

        self.context = rs.context()
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        self.align = rs.align(rs.stream.color)

        ## RM self.clipping_distance_in_meters = 2.2
        ## MB LiDar poziomo (odległość fizyczna 2.9): 2.8
        ## MB LiDar pionowo (odległość fizyczna 1.5): 1.5
        self.clipping_distance_in_meters = 1.6
        self.markers_detector = md.MarkersDetector()

    def get_device_serials(self):
        self.device_serials.clear()
        for dev in self.context.devices:
            if dev.get_info(rs.camera_info.name).lower() != 'platform camera':
                serial = dev.get_info(rs.camera_info.serial_number)
                self.device_serials.append(serial)
        print("Connected devices: {0}".format(self.device_serials))

    def devices_hardware_reset(self):
        for dev in self.context.devices:
            if dev.get_info(rs.camera_info.name).lower() != 'platform camera':
                dev.hardware_reset()
                print("Device {0} hardware reset done.".format(dev.get_info(rs.camera_info.serial_number)))

    def configure_gpio(self):
        self.cameras_gpio.clear()
        for i in range(0, len(self.device_serials)):
            serial = self.device_serials[i]
            gpio = self.gpio_config.get(serial, None)
            if gpio is None:
                log.info("Skipping connected device:" + serial)
                continue
            else:
                self.cameras_gpio[serial] = digitalio.DigitalInOut(gpio)
                self.cameras_gpio.get(serial).direction = digitalio.Direction.OUTPUT
        print("GPIO configured.")

    def set_all_cameras_flag(self, flag):
        # Set to hold all devices
        for nr in self.cameras_gpio:
            self.cameras_gpio[nr].value = flag

    def start_streaming(self):
        self.device_serials.clear()
        self.pipeline.clear()
        self.config.clear()
        self.profile.clear()
        self.depth_sensors.clear()
        self.depth_scale.clear()
        self.tt_1.clear()
        self.intr_1.clear()
        self.pinhole_camera_params.clear()

        try:
            self.devices_hardware_reset()
            time.sleep(5)
            self.get_device_serials()
            self.configure_gpio()
            self.set_all_cameras_flag(True)

            # Loop over connected devices to create config, profile and pipeline for all devices
            for serial in self.cameras_gpio:
                print("Device {0} start config.".format(serial))
                self.config[serial] = rs.config()
                self.pipeline[serial] = rs.pipeline()
                # Create a config and configure the pipeline to stream
                #  different resolutions of color and depth streams
                self.config.get(serial).enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
                self.config.get(serial).enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                print("Device {0} streams enabled.".format(serial))

                # Enable devices
                self.config.get(serial).enable_device(serial)
                print("Device {0} enabled.".format(serial))
                self.profile[serial] = self.pipeline.get(serial).start(self.config.get(serial))
                print("Device {0} streaming started.".format(serial))

                # get depth scale
                self.depth_sensors[serial] = self.profile[serial].get_device().first_depth_sensor()
                self.depth_scale[serial] = self.depth_sensors.get(serial).get_depth_scale()

                # Set slave mode
                self.depth_sensors.get(serial).set_option(rs.option.inter_cam_sync_mode, 1)
                print("Device {0} slave mode enabled.".format(serial))

                # Using preset HighAccuracy for recording
                ## RM self.depth_sensors[dev_index].set_option(rs.option.visual_preset, Preset.HighAccuracy)
                self.depth_sensors.get(serial).set_option(rs.option.visual_preset, Preset.HighAccuracy)
                print("Device {0} option set Preset.HighAccuracy.".format(serial))

                # Get intrinsic for Open3d visualization
                self.tt_1[serial] = self.profile.get(serial).get_stream(rs.stream.depth)
                self.intr_1[serial] = self.tt_1.get(serial).as_video_stream_profile().get_intrinsics()
                print("Camera {0}, video intrinsic: {1}".format(serial, self.intr_1))
                # setting intrinsic matrix from sensors_config
                intrinsic = o3d.camera.PinholeCameraIntrinsic(
                    self.sensors_config[serial]["width"],
                    self.sensors_config[serial]["height"],
                    self.sensors_config[serial]["fx"],
                    self.sensors_config[serial]["fy"],
                    self.sensors_config[serial]["cx"],
                    self.sensors_config[serial]["cy"])
                self.pinhole_camera_params[serial] = o3d.camera.PinholeCameraParameters()
                self.pinhole_camera_params[serial].intrinsic = intrinsic
                self.pinhole_camera_params[serial].extrinsic = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])
                print("Camera {0}, depth intrinsic: {1}".format(serial,
                        self.pinhole_camera_params[serial].intrinsic.intrinsic_matrix))
                # Getting the depth sensor's depth scale (see rs-align example for explanation)
                # We will not display the background of objects more than
                # clipping_distance_in_meters meters away
                self.clipping_distance[serial] = self.clipping_distance_in_meters / self.depth_scale[serial]

                # Create camera object in the Markers Detector. Intrinsic matrix from JSON!!!
                self.markers_detector.add_camera(md.Camera(serial, intrinsic.intrinsic_matrix, depth_scale=4000))
        except:
             print(traceback.format_exc())
             exit()
        print("Streams started.")

    def read_scene(self, count):
        print("Reading {0} collections of {1} pointclouds.".format(count, len(self.cameras_gpio)))
        # main loop - reading count number of pointclouds
        result = []
        i = 0  # index counter for while loop
        while i < count:
            for serial in self.gpio_config:
                # Get camera object for detection
                camera = self.markers_detector.get_camera(serial)
                if camera is None:
                    log.error("No camera found for serial {1}.".format(serial))
                    continue
                # Set active camera
                self.cameras_gpio.get(serial).value = False
                try:
                    # Get frameset of color and depth
                    for i in range(10):
                        frames = self.pipeline[serial].wait_for_frames()

                    # Align the depth frame to color frame
                    aligned_frames = self.align.process(frames)

                    # Get aligned frames
                    depth_frame = aligned_frames.get_depth_frame()
                    color_frame = aligned_frames.get_color_frame()

                    # Validate that both frames are valid
                    if not depth_frame or not color_frame:
                        continue

                    # Set inactive camera
                    self.cameras_gpio[serial].value = True

                    # Create RGBD
                    color_raw_array = np.array(color_frame.get_data())
                    # Switch RED with BLUE
                    color_raw_array[:, :, [2, 0]] = color_raw_array[:, :, [0, 2]]
                    depth_raw_array = np.array(depth_frame.get_data())
                    color_raw = o3d.geometry.Image(color_raw_array)
                    depth_raw = o3d.geometry.Image(depth_raw_array)
                    o3d.io.write_image(serial + ".jpg", color_raw)
                    ## RM depth_scale=4000,
                    ## MB LiDar poziomo (odległość fizyczna 2.9): 4500
                    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw,
                                                                                    depth_scale=4000,
                                                                                    depth_trunc=self.clipping_distance_in_meters,
                                                                                    convert_rgb_to_intensity=False)
                    # # Create Point cloud from rgbd
                    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                                         self.pinhole_camera_params[serial].intrinsic,
                                                                         self.pinhole_camera_params[serial].extrinsic)

                    log.info("Camera {0} detection start.".format(serial))
                    markers = camera.detect_markers(color_raw_array, depth_raw_array)
                    log.info("Camera {0} detection stop.".format(serial))
                    # pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_raw,
                    #                                                       self.pinhole_camera_params[serial].intrinsic,
                    #                                                       depth_scale=4000,
                    #                                                       depth_trunc=self.clipping_distance_in_meters)
                    if pcd.is_empty():
                        continue
                    result.append(sc.Scene(serial, pcd, color_raw, markers))
                except:
                    print(traceback.format_exc())
                    self.pipeline[serial].stop()
            # end of for loop
        # end of while loop
        return result

    def stop_streaming(self):
        for serial in self.pipeline:
            self.pipeline[serial].stop()
        print("Devices' pipelines stopped.")
