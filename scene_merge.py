import open3d as o3d
import numpy as np
import copy

from scene_reader import *
from marker_detector import *
import logging as log

log.basicConfig(level=log.INFO)
log = log.getLogger("scene_merge")


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def merge_markers(source_markers, target_markers, transformation_matrix):
    result = copy.deepcopy(target_markers)
    # Copy markers from source ONLY not visible in target and transform
    for marker in source_markers:
        if marker not in target_markers:
            marker.transform(transformation_matrix)
            result.append(marker)
    return result


def merge_two_scenes(source_scene, target_scene):
    # Matrix no of points x 2
    correspondence_matrix = []

    for source_scene_marker in source_scene.markers.values():
        target_scene_marker = target_scene.markers.get(source_scene_marker.id, None)
        if target_scene_marker is None:
            log.info("Marker id: {0} not found in scene {1}"
                     .format(source_scene_marker.id, target_scene.id))
            continue
        # Two the same id markers found
        log.info("Two markers id: {0} found.".format(source_scene_marker.id))
        source_point_index = source_scene.get_point_index_closest_to_marker(source_scene_marker)
        target_point_index = target_scene.get_point_index_closest_to_marker(target_scene_marker)

        # Add correspondence pair [source, target]
        correspondence_matrix.append([source_point_index[0], target_point_index[0]])

    # estimate rough transformation using correspondences
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source_scene.pointcloud,
                                            target_scene.pointcloud,
                                            o3d.utility.Vector2iVector(correspondence_matrix))

    # point-to-point ICP for refinement
    log.info("Perform point-to-point ICP refinement")
    threshold = 0.01  # 1cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_scene.pointcloud,
        target_scene.pointcloud,
        threshold,
        trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    # check if pcd matched correctly
    # draw_registration_result(source_scene.pointcloud,
    #                          target_scene.pointcloud,
    #                          reg_p2p.transformation)

    log.info("Calculated transformation: {}".format(reg_p2p.transformation))
    return reg_p2p.transformation


def merge_scenes(scenes):
    if len(scenes) < 2:
        log.error("No enough scenes to combine.")
        return

    target = copy.deepcopy(scenes[0])
    # identity transformation
    T = [np.eye(4)]
    for i in range(1, len(scenes)):
        transformation_matrix = merge_two_scenes(scenes[i], target)
        # Add transformation matrix
        T.append(transformation_matrix)
        log.info("Transformation matrices: {}".format(T))

        # transform source and add to the target
        scene_transformed = copy.deepcopy(scenes[i])
        scene_transformed.transform(transformation_matrix)
        target = target + scene_transformed

    # For test only
    # o3d.visualization.draw_geometries([target])

    return target, T

