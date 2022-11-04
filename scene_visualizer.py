import copy
import glob
from functools import partial

import logging as log
import numpy as np
import open3d as o3d
from enum import Enum
import scene as sc
import scene_reader as sr
import scene_merge as sm


sphere_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1]]


class View(Enum):
    PCD = 1
    MESH = 2


# customise open3d Visualizer
class SceneVisualizer:
    def __init__(self, lidar, input_folder):
        self.FILE_NAME = "points"
        self.MARKER_FILE_NAME = "markers"
        self.IMAGE_FILE_NAME = "image"
        self.MESH_FILE_NAME = "model"
        self.FILE_EXT = "ply"
        self.MARKER_FILE_EXT = "json"
        self.IMAGE_FILE_EXT = "png"
        self.FILE_COUNTER = 1
        self.MESH_FILE_EXT = "obj"
        self.MESH_FILE_COUNTER = 1
        self.MOVE_STEP = 0.01  # [m]
        self.ROTATE_STEP = np.pi / 180  # [rad]

        self.input_folder = input_folder
        self.lidar = lidar

        # Scenes consist of number of geometries (usually 4)
        self.scenes = []
        self.combined_scene = None

        # Geometries added (pointclouds)
        self.geometries = []
        self.colors = []
        self.geometries_colors = []
        self.active_geometries = [True, True, True, True, True]

        # Geometries for detected markers
        self.marker_spheres_geometries = []

        # combined mesh
        self.mesh = None

        self.active_view = View.PCD

        # Scene reader
        self.scene_reader = sr.SceneReader()

        # Visualizer
        self.vis = o3d.visualization.VisualizerWithKeyCallback()

        # Transformation matrices
        self.T = []

        # translate/move step homogenous transformation matrices
        self.t_px = np.eye(4)
        self.t_px[0][3] = self.MOVE_STEP
        self.t_nx = np.eye(4)
        self.t_nx[0][3] = -1 * self.MOVE_STEP

        self.t_py = np.eye(4)
        self.t_py[1][3] = self.MOVE_STEP
        self.t_ny = np.eye(4)
        self.t_ny[1][3] = -1 * self.MOVE_STEP

        self.t_pz = np.eye(4)
        self.t_pz[2][3] = self.MOVE_STEP
        self.t_nz = np.eye(4)
        self.t_nz[2][3] = -1 * self.MOVE_STEP

        # rotation homogenous transformation matrices
        self.Rxp = [[1, 0, 0, 0],
                    [0, np.cos(self.ROTATE_STEP), np.sin(self.ROTATE_STEP), 0],
                    [0, -1 * np.sin(self.ROTATE_STEP), np.cos(self.ROTATE_STEP), 0],
                    [0, 0, 0, 1]]

        self.Rxn = [[1, 0, 0, 0],
                    [0, np.cos(-1 * self.ROTATE_STEP), np.sin(-1 * self.ROTATE_STEP), 0],
                    [0, -1 * np.sin(-1 * self.ROTATE_STEP), np.cos(-1 * self.ROTATE_STEP), 0],
                    [0, 0, 0, 1]]

        self.Ryp = [[np.cos(self.ROTATE_STEP), 0, -1 * np.sin(self.ROTATE_STEP), 0],
                    [0, 1, 0, 0],
                    [np.sin(self.ROTATE_STEP), 0, np.cos(self.ROTATE_STEP), 0],
                    [0, 0, 0, 1]]

        self.Ryn = [[np.cos(-1 * self.ROTATE_STEP), 0, -1 * np.sin(-1 * self.ROTATE_STEP), 0],
                    [0, 1, 0, 0],
                    [np.sin(-1 * self.ROTATE_STEP), 0, np.cos(-1 * self.ROTATE_STEP), 0],
                    [0, 0, 0, 1]]

        self.Rzp = [[np.cos(self.ROTATE_STEP), -1 * np.sin(self.ROTATE_STEP), 0, 0],
                    [np.sin(self.ROTATE_STEP), np.cos(self.ROTATE_STEP), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]

        self.Rzn = [[np.cos(-1 * self.ROTATE_STEP), -1 * np.sin(-1 * self.ROTATE_STEP), 0, 0],
                    [np.sin(-1 * self.ROTATE_STEP), np.cos(-1 * self.ROTATE_STEP), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]

    def move_positive_x(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.t_px)
                self.T[i] = np.dot(self.t_px, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def move_negative_x(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.t_nx)
                self.T[i] = np.dot(self.t_nx, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def move_positive_y(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.t_py)
                self.T[i] = np.dot(self.t_py, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def move_negative_y(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.t_ny)
                self.T[i] = np.dot(self.t_ny, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def move_positive_z(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.t_pz)
                self.T[i] = np.dot(self.t_pz, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def move_negative_z(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.t_nz)
                self.T[i] = np.dot(self.t_nz, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def rotate_positive_x(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.Rxp)
                self.T[i] = np.dot(self.Rxp, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def rotate_negative_x(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.Rxn)
                self.T[i] = np.dot(self.Rxn, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def rotate_positive_y(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.Ryp)
                self.T[i] = np.dot(self.Ryp, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def rotate_negative_y(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.Ryn)
                self.T[i] = np.dot(self.Ryn, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def rotate_positive_z(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.Rzp)
                self.T[i] = np.dot(self.Rzp, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def rotate_negative_z(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                self.geometries[i].transform(self.Rzn)
                self.T[i] = np.dot(self.Rzn, self.T[i])
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def back_transformation(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                b_T = np.linalg.inv(self.T[i])
                self.T[i] = np.eye(4)
                self.geometries[i].transform(b_T)
                self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    def print_T(self, vis):
        print(self.T)

    def save_T(self, vis):
        with open('transform_matrices.npy', 'wb') as f:
            np.save(f, self.T)
        print("Tranformation {0} matrices saved.".format(len(self.geometries)))
        print(self.T)

    def load_T(self, vis):
        with open('transform_matrices.npy', 'rb') as f:
            self.T = np.load(f)
        print("Tranformation matrix loaded.")
        print(self.T)
        for i in range(0, len(self.T)):
            self.geometries[i].transform(self.T[i])
            self.marker_spheres_geometries[i].transform(self.T[i])
            self.vis.update_geometry(self.geometries[i])
        self.vis.update_renderer()

    # def next_active_geometry(self, vis):
    #     if self.active_geometry+1 < len(self.geometries):
    #         self.active_geometry = self.active_geometry + 1
    #     else:
    #         self.active_geometry = -1
    #     print("Active geometry = {0}".format(self.active_geometry))
    #
    # def previouse_active_geometry(self, vis):
    #     if self.active_geometry > -1:
    #         self.active_geometry = self.active_geometry - 1
    #     else:
    #         self.active_geometry = len(self.geometries) - 1
    #     print("Active geometry = {0}".format(self.active_geometry))

    def flip_i_geometry(self, flag, index):
        self.active_geometries[index] = flag
        if not self.active_geometries[index]:
            self.geometries[index].paint_uniform_color([1, 0.706, 0])
        else:
            self.geometries[index].colors = self.geometries_colors[index]
        self.vis.update_geometry(self.geometries[index])
        self.vis.update_renderer()
        print("Active geometries = {0}".format(self.active_geometries))

    def flip_0_geometry(self, vis):
        self.flip_i_geometry(not self.active_geometries[0], 0)

    def flip_1_geometry(self, vis):
        self.flip_i_geometry(not self.active_geometries[1], 1)

    def flip_2_geometry(self, vis):
        self.flip_i_geometry(not self.active_geometries[2], 2)

    def flip_3_geometry(self, vis):
        self.flip_i_geometry(not self.active_geometries[3], 3)

    def flip_4_geometry(self, vis):
        self.flip_i_geometry(not self.active_geometries[4], 4)

    def set_all_geometries(self, vis):
        for i in range(0, len(self.active_geometries)):
            self.active_geometries[i] = True
            self.flip_i_geometry(self.active_geometries[i], i)
        print("Active geometries = {0}".format(self.active_geometries))

    def export_points_and_images(self, vis):
        # for i in range(0, len(self.geometries)):
        #     file_name = self.FILE_NAME + str(self.FILE_COUNTER) + '_' + str(i) + '.' + self.FILE_EXT
        #     o3d.io.write_point_cloud(file_name, self.geometries[i])
        #     print("Point could exported to file {0}".format(file_name))
        #     if len(self.colors) > i:
        #         image_file_name = self.IMAGE_FILE_NAME + str(self.FILE_COUNTER) + '_' + str(i) + '.' + self.IMAGE_FILE_EXT
        #         o3d.io.write_image(image_file_name, self.colors[i])

        file_name = self.FILE_NAME + str(self.FILE_COUNTER) + '.' + self.FILE_EXT
        self.calculate_combined_scene()
        o3d.io.write_point_cloud(file_name, self.combined_scene.pointcloud)
        print("Combined point could exported to file: {0}".format(file_name))
        marker_file_name = self.MARKER_FILE_NAME + str(self.FILE_COUNTER) + '.' + self.MARKER_FILE_EXT
        with open(marker_file_name, "w") as outfile:
            outfile.write(self.combined_scene.markers_to_json())

        print("Markers exported to file: {0}".format(marker_file_name))
        self.FILE_COUNTER += 1

    def down_sample(self, vis):
        for index, scene in enumerate(self.scenes):
            pcd = scene.pointcloud.voxel_down_sample(voxel_size=0.005)
            scene.pointcloud.points = pcd.points
            scene.pointcloud.colors = pcd.colors

        self.update_geometries()
        log.info("Pointclouds down sampled")
        # for i in range(0, len(self.geometries)):
        #     if self.active_geometries[i]:
        #         pcd = self.geometries[i].voxel_down_sample(voxel_size=0.005)
        #         self.geometries[i].points = pcd.points
        #         self.geometries[i].colors = pcd.colors
        #         self.vis.update_geometry(self.geometries[i])
        # self.vis.update_renderer()
        # print("Active geometry {0} down sampled".format(self.active_geometries))

    def filter_radius(self, vis):
        for index, scene in enumerate(self.scenes):
            pcd, _ = scene.pointcloud.remove_radius_outlier(nb_points=100, radius=0.1)
            scene.pointcloud.points = pcd.points
            scene.pointcloud.colors = pcd.colors

        self.update_geometries()
        log.info("Pointclouds filtered by radius")

        # for i in range(0, len(self.geometries)):
        #     if self.active_geometries[i]:
        #         pcd, _ = self.geometries[i].remove_radius_outlier(nb_points=100, radius=0.05)
        #         self.geometries[i].points = pcd.points
        #         self.geometries[i].colors = pcd.colors
        #         self.vis.update_geometry(self.geometries[i])
        # self.vis.update_renderer()
        # print("Active geometry {0} filtered by radius".format(self.active_geometries))

    def remove_markers(self, vis):
        for scene in self.scenes:
            scene.remove_color_markers()

        self.update_geometries()
        log.info("Color ball markers removed")

    def create_mesh(self, vis):
        pcd = self.calculate_combined_scene()
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            self.mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                pcd, depth=15, width=0, scale=1.1, linear_fit=False)

        # radii = [0.005, 0.01, 0.02, 0.04]
        # self.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        #     pcd, o3d.utility.DoubleVector(radii))
        # alpha = 0.03
        # print(f"alpha={alpha:.3f}")
        # self.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

        print(self.mesh)

    def switch_geometries(self, vis):
        self.vis.clear_geometries()
        if self.active_view is View.PCD:
            self.active_view = View.MESH
            if self.mesh is None:
                self.create_mesh(self.vis)
            self.vis.add_geometry(self.mesh)
            print("Geometry switched to MESH")
        else:
            for i in range(0, len(self.geometries)):
                self.active_view = View.PCD
                self.vis.add_geometry(self.geometries[i])
            print("Geometry switched to PCD")
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        self.vis.add_geometry(axis)
        self.vis.update_renderer()

    def export_mesh(self, vis):
        file_name = self.MESH_FILE_NAME + str(self.MESH_FILE_COUNTER) + '.' + self.MESH_FILE_EXT
        self.create_mesh(vis)
        o3d.io.write_triangle_mesh(filename=file_name, mesh=self.mesh, write_ascii=True, print_progress=True)
        self.MESH_FILE_COUNTER += 1
        print("Combined point could exported to file {0}".format(file_name))

    def reset_camera(self, vis):
        self.vis.reset_view_point(True)

    def estimate_normals_for_geometry(self, vis):
        for i in range(0, len(self.geometries)):
            if self.active_geometries[i]:
                if self.geometries[i].has_normals():
                    self.geometries[i].orient_normals_towards_camera_location()
                else:
                    self.geometries[i].estimate_normals()
        print("Normals estimated for geometry {0}".format(self.active_geometries))

    def calculate_combined_scene(self):
        self.combined_scene = self.scenes[0]
        for i in range(1, len(self.scenes)):
            self.combined_scene += self.scenes[i]

        self.combined_scene.update_index_of_markers()
        return

    def read_scenes_from_files(self):
        files = glob.glob("./" + self.input_folder + "/*." + self.FILE_EXT)
        order = [0, 1, 2, 3, 4]
        neg_suffix_length = -1 * len("." + self.FILE_EXT)
        scenes = []
        for index in order:
            for file in files:
                if file[neg_suffix_length-1:neg_suffix_length] == str(index):
                    pcd = o3d.io.read_point_cloud(file)
                    scenes.append(sc.Scene(index, pcd, pcd.colors, None))
                    print("File: " + file + " read.")
                    break
        return scenes

    def add_all_geometries(self):
        # Add all geometries to the Visualizer
        for i in range(0, len(self.geometries)):
            self.geometries[i].transform(self.T[i])
            # self.marker_spheres[i].transform(self.T[i])
            self.vis.add_geometry(self.geometries[i])

        # Add all markers to the Visualizer
        for i in range(0, len(self.marker_spheres_geometries)):
            self.vis.add_geometry(self.marker_spheres_geometries[i])

    def update_geometries(self):
        self.vis.clear_geometries()
        self.geometries.clear()
        self.marker_spheres_geometries.clear()

        # Add all scenes to the geometries table
        for index, scene in enumerate(self.scenes):
            self.geometries.append(scene.pointcloud)
            self.geometries_colors.append(copy.deepcopy(scene.colors))

            for marker_id in scene.markers:
                marker = scene.markers.get(marker_id)
                print("Scene {0} marker {1} position: x:{2}, y:{3}, z:{4}"
                      .format(scene.id, marker_id, marker.x, marker.y, marker.z))
                # index = scene.get_point_index_closest_to_marker(marker)
                # i = index[0]
                # print("Index of the closest point = {0}. Position: {1}".
                #       format(i, scene.pointcloud.points[i]))
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
                sphere.paint_uniform_color(sphere_colors[index])
                transform_matrix = marker.get_position_as_transformation_matrix()
                sphere.transform(transform_matrix)
                # print("Marker {0} position {1}.".format(marker_id, transform_matrix))
                self.marker_spheres_geometries.append(sphere)

        self.add_all_geometries()

        # Optionally add coordinate axis
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        self.vis.add_geometry(axis)

    def read_update_scene(self, vis):
        self.scenes.clear()
        self.scenes = self.scene_reader.read_scene(1)

        if len(self.scenes) < 1:
            print("Too few scenes.")
            return

        # Add initial transformation matrix for the geometry
        for i in range(0, len(self.scenes)):
            self.T.append(np.eye(4))

        self.update_geometries()

    def merge(self, vis):
        result, T = sm.merge_scenes(self.scenes)
        self.combined_scene = result
        for i in range(1, len(self.scenes)):
            self.scenes[i].transform(T[i])

        self.update_geometries()

    def display(self):
        if self.lidar:
            self.scene_reader.start_streaming()
            self.vis.create_window(width=1280, height=720)
            self.read_update_scene(vis=self.vis)
        else:
            self.scenes = self.read_scenes_from_files()

        # Auto transformation calculation
        # with open('auto_transform_matrices.npy', 'rb') as f:
        #     self.T = np.load(f)
        # print("Auto tranformation matrix loaded.")

        print("Active geometries = {0}".format(self.active_geometries))

        # code https://www.glfw.org/docs/latest/group__keys.html
        self.vis.register_key_callback(297, partial(self.merge))  # F8
        self.vis.register_key_callback(296, partial(self.remove_markers))  # F7

        self.vis.register_key_callback(65, partial(self.move_negative_x))  # A
        self.vis.register_key_callback(68, partial(self.move_positive_x))  # D
        self.vis.register_key_callback(87, partial(self.move_positive_y))  # W
        self.vis.register_key_callback(83, partial(self.move_negative_y))  # S
        self.vis.register_key_callback(90, partial(self.move_negative_z))  # Z
        self.vis.register_key_callback(88, partial(self.move_positive_z))  # X

        self.vis.register_key_callback(298, partial(self.read_update_scene))  # F9
        self.vis.register_key_callback(82, partial(self.rotate_negative_x))  # R
        self.vis.register_key_callback(84, partial(self.rotate_positive_x))  # T
        self.vis.register_key_callback(70, partial(self.rotate_negative_y))  # F
        self.vis.register_key_callback(71, partial(self.rotate_positive_y))  # G
        self.vis.register_key_callback(86, partial(self.rotate_negative_z))  # V
        self.vis.register_key_callback(66, partial(self.rotate_positive_z))  # B
        self.vis.register_key_callback(259, partial(self.back_transformation))  # backspace
        #self.vis.register_key_callback(80, partial(self.print_T))  # P
        self.vis.register_key_callback(79, partial(self.save_T))  # O
        self.vis.register_key_callback(76, partial(self.load_T))  # L
        # self.vis.register_key_callback(266, partial(self.next_active_geometry))  # PAGE_UP
        # self.vis.register_key_callback(267, partial(self.previouse_active_geometry))  # PAGE_DOWN
        self.vis.register_key_callback(73, partial(self.export_points_and_images))  # I  export ply
        self.vis.register_key_callback(46, partial(self.down_sample))  # .  voxel down sample
        self.vis.register_key_callback(44, partial(self.filter_radius))  # ,  filter by radius
        self.vis.register_key_callback(77, partial(self.create_mesh))  # M
        self.vis.register_key_callback(47, partial(self.switch_geometries))  # /
        self.vis.register_key_callback(85, partial(self.export_mesh))  # U
        self.vis.register_key_callback(69, partial(self.reset_camera))  # E
        self.vis.register_key_callback(67, partial(self.estimate_normals_for_geometry))  # C

        self.vis.register_key_callback(290, partial(self.flip_0_geometry))    # F1
        self.vis.register_key_callback(291, partial(self.flip_1_geometry))    # F2
        self.vis.register_key_callback(292, partial(self.flip_2_geometry))    # F3
        self.vis.register_key_callback(293, partial(self.flip_3_geometry))    # F4
        self.vis.register_key_callback(294, partial(self.flip_4_geometry))  # F5
        self.vis.register_key_callback(301, partial(self.set_all_geometries))    # F12

        self.vis.run()
        self.scene_reader.stop_streaming()
        print("Vis window closed")
        self.vis.destroy_window()
