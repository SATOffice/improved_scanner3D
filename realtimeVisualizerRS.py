import sys

import pyrealsense2 as rs
import numpy as np
import threading as th

import open3d as o3d
import marker_detector as md
import scene as sc

threadLock = th.Lock()
keep_running = True
geometry_added = False
vis = o3d.visualization.Visualizer()
vis.create_window(width=1280, height=720)
serial = "337"


# L515 reading data thread
class ReadDataThread(th.Thread):
    def __init__(self, threadID, name, counter):
        th.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter

        # Init RS streams
        self.config = rs.config()
        self.pipeline = rs.pipeline()
        self.config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.profile = None
        self.depth_sensor = None
        self.pinhole_camera_intrinsic = None
        self.depth_scale = 4000
        self.image_array = []
        self.depth_array = []
        self.pointcloud = o3d.geometry.PointCloud()

        self.spheres = {}
        self.markers_detector = md.MarkersDetector()
        self.arucos_last_position = {}
        self.scene = None

    def run(self):
        print("Starting " + self.name)
        global pointcloud
        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        # Align setup depth to color stream
        align_to = rs.stream.color
        align = rs.align(align_to)
        # Get intrinsic
        tt_1 = self.profile.get_stream(rs.stream.depth)
        intr_1 = tt_1.as_video_stream_profile().get_intrinsics()
        self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intr_1.width, intr_1.height, intr_1.fx,
                                                                          intr_1.fy, intr_1.ppx, intr_1.ppy)
        lidar = md.Camera(serial, self.pinhole_camera_intrinsic.intrinsic_matrix, depth_scale=4000)
        self.markers_detector.add_camera(lidar)

        # Thread main loop - keep_running should be change in the main thread
        while keep_running:
            try:
                # Wait for the next set of frames from the camera
                frames = self.pipeline.wait_for_frames()
                # Align depth and color frame
                aligned_frames = align.process(frames)

                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                # if not depth_frame or not depth_frame:
                #    continue
                # Create RGBD
                color_raw_array = np.array(color_frame.get_data())
                color_raw_array[:, :, [2, 0]] = color_raw_array[:, :, [0, 2]]
                depth_raw_array = np.array(depth_frame.get_data())
                color_raw = o3d.geometry.Image(color_raw_array)
                depth_raw = o3d.geometry.Image(depth_raw_array)

                rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw,
                                                                                depth_scale=self.depth_scale,
                                                                                depth_trunc=2,
                                                                                convert_rgb_to_intensity=False)

                # Create Point cloud from rgbd
                pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, self.pinhole_camera_intrinsic)

                markers = lidar.detect_auroc_markers(color_raw_array, depth_raw_array)

                threadLock.acquire()
                self.image_array = color_raw_array
                self.depth_array = depth_raw_array
                self.pointcloud.points = pcd.points
                self.pointcloud.colors = pcd.colors
                self.scene = sc.Scene(serial, pcd, color_raw, markers)
                threadLock.release()
            except:
                e = sys.exc_info()
                print(e)
        # end mail loop
        self.pipeline.stop()
        print("Stopping " + self.name)


# try:
readThread = ReadDataThread(1, "L515 read", 1)

readThread.start()
while keep_running:
    threadLock.acquire()
    if len(readThread.image_array) == 0:
        # print("image2D is empty. Continue.")
        threadLock.release()
        continue
    scene = readThread.scene
    if scene is None:
        keep_running = False
        continue

    for marker in scene.markers.values():
        # Get last position of the given marker
        id_last_position = readThread.arucos_last_position.get(marker.id, None)
        if id_last_position is None:
            id_last_position = np.zeros(4)
            readThread.arucos_last_position[marker.id] = id_last_position

        # Get geometry for the marker
        sphere = readThread.spheres.get(marker.id, None)
        if sphere is None:
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
            sphere.paint_uniform_color([255, 0, 0])
            vis.add_geometry(sphere)
            readThread.spheres[marker.id] = sphere
        # Transformation matrix calculation
        marker_position = marker.get_position_as_vector()
        new_position = np.zeros((4, 3))
        new_position = np.append(new_position, marker_position, axis=1)
        aruco_move = np.eye(4) + new_position - id_last_position
        id_last_position = new_position
        readThread.arucos_last_position[marker.id] = id_last_position
        sphere.transform(aruco_move)
        vis.update_geometry(sphere)

    # image = o3d.geometry.Image(readThread.depth_array)
    # vis.clear_geometries()
    # vis.add_geometry(image)
    if not geometry_added:
        vis.add_geometry(readThread.pointcloud)
        # vis.add_geometry(readThread.single_image)
        geometry_added = True
    else:
        vis.update_geometry(readThread.pointcloud)
    threadLock.release()

    keep_running = keep_running and vis.poll_events()
    vis.update_renderer()
    # o3d.visualization.draw_geometries([pointcloud],
    #                                   zoom=0.3412,
    #                                   front=[0.4257, -0.2125, -0.8795],
    #                                   lookat=[2.6172, 2.0475, 1.532],
    #                                   up=[-0.0694, -0.9768, 0.2024])
print("Vis window closed")
vis.destroy_window()
# finally:
#     print("Main thread finished")
