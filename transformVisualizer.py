from functools import partial

import sys
import pyrealsense2 as rs
import numpy as np
import open3d as o3d

MOVE_STEP = 0.01  # [m]
ROTATE_STEP = np.pi / 180  # [rad]

pointcloud = o3d.geometry.PointCloud()
T = np.eye(4)

t_px_c = 0
t_nx_c = 0
t_py_c = 0
t_ny_c = 0
t_pz_c = 0
t_nz_c = 0

r_px_c = 0
r_nx_c = 0
r_py_c = 0
r_ny_c = 0
r_pz_c = 0
r_nz_c = 0

# translate x y z matrices for one step
t_px = np.eye(4)
t_px[0][3] = MOVE_STEP
t_nx = np.eye(4)
t_nx[0][3] = -1 * MOVE_STEP

t_py = np.eye(4)
t_py[1][3] = MOVE_STEP
t_ny = np.eye(4)
t_ny[1][3] = -1 * MOVE_STEP

t_pz = np.eye(4)
t_pz[2][3] = MOVE_STEP
t_nz = np.eye(4)
t_nz[2][3] = -1 * MOVE_STEP

# rotation matrices for one step

Rxp = [[1, 0, 0, 0],
       [0, np.cos(ROTATE_STEP), np.sin(ROTATE_STEP), 0],
       [0, -1*np.sin(ROTATE_STEP), np.cos(ROTATE_STEP), 0],
       [0, 0, 0, 1]]

Rxn = [[1, 0, 0, 0],
       [0, np.cos(-1*ROTATE_STEP), np.sin(-1*ROTATE_STEP), 0],
       [0, -1*np.sin(-1*ROTATE_STEP), np.cos(-1*ROTATE_STEP), 0],
       [0, 0, 0, 1]]

Ryp = [[np.cos(ROTATE_STEP), 0, -1*np.sin(ROTATE_STEP), 0],
       [0, 1, 0, 0],
       [np.sin(ROTATE_STEP), 0, np.cos(ROTATE_STEP), 0],
       [0, 0, 0, 1]]

Ryn = [[np.cos(-1*ROTATE_STEP), 0, -1*np.sin(-1*ROTATE_STEP), 0],
       [0, 1, 0, 0],
       [np.sin(-1*ROTATE_STEP), 0, np.cos(-1*ROTATE_STEP), 0],
       [0, 0, 0, 1]]

Rzp = [[np.cos(ROTATE_STEP), -1*np.sin(ROTATE_STEP), 0, 0],
       [np.sin(ROTATE_STEP), np.cos(ROTATE_STEP), 0, 0],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]

Rzn = [[np.cos(-1*ROTATE_STEP), -1*np.sin(-1*ROTATE_STEP), 0, 0],
       [np.sin(-1*ROTATE_STEP), np.cos(-1*ROTATE_STEP), 0, 0],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]


vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window(width=1280, height=720)


# L515 reading data thread
class ReadData:
    def __init__(self):
        # Init RS streams
        self.config = rs.config()
        self.pipeline = rs.pipeline()
        self.config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.profile = None
        self.depth_sensor = None

    def read(self, count):
        print("Read {0} frames".format(count))
        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        # Align setup depth to color stream
        align_to = rs.stream.color
        align = rs.align(align_to)
        # Get intrinsic
        tt_1 = self.profile.get_stream(rs.stream.depth)
        intr_1 = tt_1.as_video_stream_profile().get_intrinsics()
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intr_1.width, intr_1.height, intr_1.fx,
                                                                     intr_1.fy, intr_1.ppx, intr_1.ppy)
        # Thread main loop - keep_running should be change in the main thread
        i = 0  # index counter for while loop
        while i < count:
            try:
                # Wait for the next set of frames from the camera
                frames = self.pipeline.wait_for_frames()
                # Align depth and color frame
                aligned_frames = align.process(frames)

                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not depth_frame:
                    continue
                # Create RGBD
                color_raw = o3d.geometry.Image(np.array(color_frame.get_data()))
                depth_raw = o3d.geometry.Image(np.array(depth_frame.get_data()))

                rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw,
                                                                                depth_scale=4000,
                                                                                depth_trunc=3,
                                                                                convert_rgb_to_intensity=False)
                # Create Point cloud from rgbd
                pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
                # pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
                if (pcd.is_empty()):
                    continue
                # increment index
                i += 1
            except:
                e = sys.exc_info()[0]
        # end mail loop
        self.pipeline.stop()
        # return pointcloud
        return pcd


def move_positive_x(vis):
    global pointcloud
    global T
    global t_px_c
    t_px_c = t_px_c + 1
    print("t_px_c = {0}".format(t_px_c))
    # pointcloud.translate((MOVE_STEP, 0, 0))
    pointcloud.transform(t_px)
    T = np.dot(t_px, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def move_negative_x(vis):
    global pointcloud
    global T
    global t_nx_c
    t_nx_c = t_nx_c + 1
    print("t_nx_c = {0}".format(t_nx_c))
    # pointcloud.translate((-1*MOVE_STEP, 0, 0))
    pointcloud.transform(t_nx)
    T = np.dot(t_nx, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def move_positive_y(vis):
    global pointcloud
    global T
    global t_py_c
    t_py_c = t_py_c + 1
    print("t_py_c = {0}".format(t_py_c))
    # pointcloud.translate((0, MOVE_STEP, 0))
    pointcloud.transform(t_py)
    T = np.dot(t_py, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def move_negative_y(vis):
    global pointcloud
    global T
    global t_ny_c
    t_ny_c = t_ny_c + 1
    print("t_ny_c = {0}".format(t_ny_c))
    # pointcloud.translate((0, -1*MOVE_STEP, 0))
    pointcloud.transform(t_ny)
    T = np.dot(t_ny, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def move_positive_z(vis):
    global pointcloud
    global T
    global t_pz_c
    t_pz_c = t_pz_c + 1
    print("t_pz_c = {0}".format(t_pz_c))
    # pointcloud.translate((0, 0, MOVE_STEP))
    pointcloud.transform(t_pz)
    T = np.dot(t_pz, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def move_negative_z(vis):
    global pointcloud
    global T
    global t_nz_c
    t_nz_c = t_nz_c + 1
    print("t_nz_c = {0}".format(t_nz_c))
    # pointcloud.translate((0, 0, -1*MOVE_STEP))
    pointcloud.transform(t_nz)
    T = np.dot(t_nz, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def rotate_positive_x(vis):
    global pointcloud
    global T
    global r_px_c
    global Rxp
    r_px_c = r_px_c + 1
    print("r_px_c = {0}".format(r_px_c))
    pointcloud.transform(Rxp)
    T = np.dot(Rxp, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def rotate_negative_x(vis):
    global pointcloud
    global T
    global r_nx_c
    global Rxn
    r_nx_c = r_nx_c + 1
    print("r_nx_c = {0}".format(r_nx_c))
    pointcloud.transform(Rxn)
    T = np.dot(Rxn, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def rotate_positive_y(vis):
    global pointcloud
    global T
    global r_py_c
    global Ryp
    r_py_c = r_py_c + 1
    print("r_py_c = {0}".format(r_py_c))
    pointcloud.transform(Ryp)
    T = np.dot(Ryp, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def rotate_negative_y(vis):
    global pointcloud
    global T
    global r_ny_c
    global Ryn
    r_ny_c = r_ny_c + 1
    print("r_ny_c = {0}".format(r_ny_c))
    pointcloud.transform(Ryn)
    T = np.dot(Ryn, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def rotate_positive_z(vis):
    global pointcloud
    global T
    global r_pz_c
    global Rzp
    r_pz_c = r_pz_c + 1
    print("r_pz_c = {0}".format(r_pz_c))
    pointcloud.transform(Rzp)
    T = np.dot(Rzp, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def rotate_negative_z(vis):
    global pointcloud
    global T
    global r_nz_c
    global Rzn
    r_nz_c = r_nz_c + 1
    print("r_nz_c = {0}".format(r_nz_c))
    pointcloud.transform(Rzn)
    T = np.dot(Rzn, T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def back_transformation(vis):
    global pointcloud
    global T
    b_T = np.linalg.inv(T)
    T = np.eye(4)
    pointcloud.transform(b_T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


def print_T(vis):
    global T
    print(T)


def save_T(vis):
    global T
    with open('transform_matrix.npy', 'wb') as f:
        np.save(f, T)
    print("Tranformation matrix saved.")
    print(T)

def load_T(vis):
    global T
    with open('transform_matrix.npy', 'rb') as f:
        T = np.load(f)
    print("Tranformation matrix loaded.")
    print(T)
    pointcloud.transform(T)
    vis.update_geometry(pointcloud)
    vis.update_renderer()


try:
    read = ReadData()
    pointcloud = read.read(1)
    vis.add_geometry(pointcloud)
    print("Geometry added")

    # code https://www.glfw.org/docs/latest/group__keys.html
    vis.register_key_callback(65, partial(move_negative_x))  # A
    vis.register_key_callback(68, partial(move_positive_x))  # D
    vis.register_key_callback(87, partial(move_positive_y))  # W
    vis.register_key_callback(83, partial(move_negative_y))  # S
    vis.register_key_callback(90, partial(move_negative_z))  # Z
    vis.register_key_callback(88, partial(move_positive_z))  # X

    vis.register_key_callback(82, partial(rotate_negative_x))  # R
    vis.register_key_callback(84, partial(rotate_positive_x))  # T
    vis.register_key_callback(70, partial(rotate_negative_y))  # F
    vis.register_key_callback(71, partial(rotate_positive_y))  # G
    vis.register_key_callback(86, partial(rotate_negative_z))  # V
    vis.register_key_callback(66, partial(rotate_positive_z))  # B
    vis.register_key_callback(259, partial(back_transformation))  # backspace
    vis.register_key_callback(80, partial(print_T))  # P
    vis.register_key_callback(79, partial(save_T))  # O
    vis.register_key_callback(76, partial(load_T))  # L

    vis.run()
    print("Vis window closed")
    vis.destroy_window()
finally:
    print("Main thread finished")
