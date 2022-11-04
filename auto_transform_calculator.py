import argparse
import copy
import glob
import numpy as np
import open3d as o3d
import multiprocessing as mp


class PickedPoints:
    def __init__(self, name, picked_points):
        self.name = name
        self.picked_points = picked_points

    def is_name(self, name):
        return self.name.lower() == name.lower()


class AutoTransformCalculator:
    def __init__(self, target, source):

        self.target = target
        self.source = source

        # Visualizers
        self.visualizers = []

    def run_vis(self, name, geometry, data_queue):
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name=name, width=1280, height=720)
        vis.add_geometry(geometry)
        vis.run()
        print("Vis {0} window closed".format(name))
        vis.destroy_window()
        data_queue.put(PickedPoints(name, vis.get_picked_points()))

    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])

    def calculate_transform(self):
        # queue to return a value from the Process
        data_queue = mp.Queue()

        # --------------------- Single or multiple processes ----------------------------------

        # Single process (use on MacOS)
        # self.run_vis("TARGET", self.target, data_queue)
        # self.run_vis("SOURCE", self.source, data_queue)
        # self.run_vis("TARGET", self.target, data_queue)
        # self.run_vis("SOURCE", self.source, data_queue)

        # Multiple processes
        target_prc = mp.Process(target=self.run_vis,
                                args=("TARGET", self.target, data_queue))

        source_prc = mp.Process(target=self.run_vis,
                                args=("SOURCE", self.source, data_queue))
        source_prc.start()
        target_prc.start()

        source_prc.join()
        target_prc.join()

        # -------------------------------------------------------------------------------------

        # Last element
        data_queue.put(None)

        target_picked_points = []
        source_picked_points = []

        for points in iter(data_queue.get, None):
            if points.is_name("target"):
                target_picked_points = points.picked_points
            if points.is_name("source"):
                source_picked_points = points.picked_points

        print("Source points: {0}".format(source_picked_points))
        print("Target points: {0}".format(target_picked_points))

        if not(len(source_picked_points) >= 3 and len(target_picked_points) >= 3):
            print("Too few selected points")
            return
        if not(len(source_picked_points) == len(target_picked_points)):
            print("No equal points count selected. Source = {0}, target = {1}".
                  format(len(source_picked_points), len(target_picked_points)))
            return

        corr = np.zeros((len(source_picked_points), 2))
        corr[:, 0] = source_picked_points
        corr[:, 1] = target_picked_points

        # estimate rough transformation using correspondences
        print("Compute a rough transform using the correspondences given by user")
        p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        trans_init = p2p.compute_transformation(self.source, self.target,
                                                o3d.utility.Vector2iVector(corr))

        # point-to-point ICP for refinement
        print("Perform point-to-point ICP refinement")
        threshold = 0.01  # 1cm distance threshold
        reg_p2p = o3d.pipelines.registration.registration_icp(
                  self.source, self.target, threshold, trans_init,
                  o3d.pipelines.registration.TransformationEstimationPointToPoint())
        # check if pcd matched correctly
        self.draw_registration_result(self.source, self.target, reg_p2p.transformation)

        # store transformation matrix
        return reg_p2p.transformation


def read_geometries_from_files(input_folder):
    files = glob.glob("./" + input_folder + "/*.ply")
    geometries_form_files = []
    for file in files:
        geometries_form_files.append(o3d.io.read_point_cloud(file))
        print("File: " + file + " read.")
    return geometries_form_files

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description=
        "MultiLidar mesh creator. Please select one of the optional arguments")
    parser.add_argument("--input_folder",
                        default='../input/',
                        help="set input folder")
    parser.add_argument("--lidar",
                        action='store_true',
                        help="Recording rgbd stream from L515")

    args = parser.parse_args()
    # main loop over geometries. The first one is TARGET
    if args.lidar:
        import scene_reader as sr
        scene_reader = sr.SceneReader()
        scene_reader.start_streaming()
        scenes, colors = scene_reader.read_scene(1)
        geometries = scenes[0]
    else:
        geometries = read_geometries_from_files(args.input_folder)

    target = geometries[0]
    # identity transformation
    T = [np.eye(4)]
    for i in range(1, len(geometries)):
        cal = AutoTransformCalculator(target, geometries[i])
        T.append(cal.calculate_transform())
        print(T)
        # transform source and add to the target
        geometries[i].transform(T[i])
        target += geometries[i]

    with open('auto_transform_matrices.npy', 'wb') as f:
        np.save(f, T)
    print("Tranformation matrices saved.")
    print(T)
    print("------------------- Auto transform calculation finished ------------------")
