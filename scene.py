import logging as log
import open3d as o3d
import numpy as np

import marker_detector

log.basicConfig(level=log.INFO)
log = log.getLogger("scene")


class Scene:
    def __init__(self, id, pointcloud, colors, markers):
        self.pointcloud = pointcloud
        self.colors = colors
        self.markers = markers
        self.id = id
        self.update_index_of_markers()

    def __add__(self, scene):
        self.pointcloud += scene.pointcloud
        for marker in scene.markers.values():
            this_scene_marker = self.markers.get(marker.id, None)
            if this_scene_marker is None:
                log.info("Marker id: {0} not found in scene {1}. Adding to the result"
                         .format(self.id, scene.id))
                self.markers[marker.id] = marker
        self.update_index_of_markers()
        return self

    def update_index_of_markers(self):
        pcd_tree = o3d.geometry.KDTreeFlann(self.pointcloud)
        for marker in self.markers.values():
            point = np.array([marker.x, marker.y, marker.z])
            [k, idx, _] = pcd_tree.search_knn_vector_3d(point, 1)
            marker.set_index(idx[0])

    def get_point_index_closest_to_position(self, point):
        pcd_tree = o3d.geometry.KDTreeFlann(self.pointcloud)
        [k, idx, _] = pcd_tree.search_knn_vector_3d(point, 1)
        return idx

    def get_point_index_closest_to_marker(self, marker):
        point = np.array([marker.x, marker.y, marker.z])
        index = self.get_point_index_closest_to_position(point)
        return index

    def transform(self, transformation_matrix):
        self.pointcloud.transform(transformation_matrix)
        for marker in self.markers.values():
            marker.transform(transformation_matrix)

    def markers_to_json(self):
        output_json = "{[\n"
        intend = "    "
        for marker in self.markers.values():
            output_json += intend + marker.to_json() + ",\n"
        output_json = output_json[:-2]
        output_json += "]}\n"
        return output_json

    def remove_color_markers(self):
        points = np.array(self.pointcloud.points)
        for marker in self.markers.values():
            if marker.marker_type is marker_detector.MarkerType.COLOR_BALL:
                # Calculate distances to center, set new points
                center = np.array([marker.x, marker.y, marker.z])
                distances = np.linalg.norm(points - center, axis=1)
                self.pointcloud.points = o3d.utility.Vector3dVector(points[distances > marker.radius])

