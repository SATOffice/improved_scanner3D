import cv2 as cv
import numpy as np
import logging as log

log.basicConfig(level=log.INFO)
log = log.getLogger("marker_detector")

colors_level = {
    'green': {'min': (51, 133, 97), 'max': (68, 255, 238)},
    'yellow': {'min': (22, 82, 129), 'max': (44, 255, 255)},
    'pink': {'min': (157, 96, 115), 'max': (179, 255, 255)},
    'blue': {'min': (90, 47, 102), 'max': (113, 175, 210)}
}


class MarkerType:
    AUROC = 1
    COLOR_BALL = 2


class Marker:
    def __init__(self, id, marker_type=MarkerType.AUROC):
        self.id = id
        self.x = 0
        self.y = 0
        self.z = 0
        self.index = 0
        self.marker_type = marker_type
        self.radius = 0

    def set_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def set_radius(self, radius):
        self.radius = radius

    def set_index(self, index):
        self.index = index

    def get_position(self):
        return self.x, self.y, self.z

    def get_position_as_vector(self):
        position = np.array([self.x, self.y, self.z, 0], ndmin=2).T
        return position

    def get_position_as_transformation_matrix(self):
        transformation_matrix = np.zeros((4, 3))
        transformation_matrix = np.append(transformation_matrix, self.get_position_as_vector(), axis=1)
        transformation_matrix = transformation_matrix + np.eye(4)
        return transformation_matrix

    def transform(self, transformation_matrix):
        result = np.matmul(transformation_matrix, self.get_position_as_transformation_matrix())
        self.x = result[0, 3]
        self.y = result[1, 3]
        self.z = result[2, 3]

    def to_json(self):
        return "{\"id\": " + str(self.id) + ",\n" + \
               "    \"x\": " + str(self.x) + ",\n" + \
               "    \"y\": " + str(self.y) + ",\n" + \
               "    \"z\": " + str(self.z) + ",\n" + \
               "    \"index\":" + str(self.index) + "}"


class MarkersDetector:
    def __init__(self):
        self.cameras = {}

    def add_camera(self, camera):
        self.cameras[camera.serial] = camera

    def get_camera(self, serial):
        camera = self.cameras.get(serial, None)
        if camera is None:
            log.error("{0}->get_camera. Can't find the camera for serial {1}.".format(MODULE, serial))
            return
        return camera


class Camera:
    def __init__(self, serial, intrinsics_matrix, depth_scale=4000):
        self.serial = serial
        self.intrinsics_matrix = intrinsics_matrix
        self.arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        self.arucoParams = cv.aruco.DetectorParameters_create()
        self.depth_scale = depth_scale

    def detect_markers(self, rgb_mage, depth_image):
        aruco_markers = self.detect_auroc_markers(rgb_mage, depth_image)
        color_markers = self.detect_color_ball_marker(rgb_mage, depth_image)
        markers = {**aruco_markers, **color_markers}
        return markers

    def calculate_xyz_from_uv(self, u, v, depth_image):
        z = depth_image[v, u] / self.depth_scale
        # [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        x = (u - self.intrinsics_matrix[0, 2]) * z / self.intrinsics_matrix[0, 0]
        y = (v - self.intrinsics_matrix[1, 2]) * z / self.intrinsics_matrix[1, 1]
        return x, y, z

    def detect_auroc_markers(self, rgb_image, depth_image):
        markers = {}
        (corners, ids, rejected) = cv.aruco.detectMarkers(rgb_image, self.arucoDict, parameters=self.arucoParams)
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # Get last position of the given marker
                marker = markers.get(markerID, None)
                # If no ID in the dictionary, then add one
                if marker is None:
                    marker = Marker(markerID)
                    markers[markerID] = marker

                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # compute and draw the center (U, V) - coordinates of the ArUco
                # marker
                U = int((topLeft[0] + bottomRight[0]) / 2.0)
                V = int((topLeft[1] + bottomRight[1]) / 2.0)

                x, y, z = self.calculate_xyz_from_uv(U, V, depth_image)
                # update markers position in the dictionary
                marker.set_position(x, y, z)
        return markers

    def detect_color_ball_marker(self, rgb_image, depth_image):
        markers = {}
        blurred = cv.GaussianBlur(rgb_image, (11, 11), 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_RGB2HSV)

        for color in colors_level:
            mask = cv.inRange(hsv, colors_level[color]['min'], colors_level[color]['max'])
            mask = cv.erode(mask, None, iterations=2)
            mask = cv.dilate(mask, None, iterations=2)
            circles = cv.HoughCircles(mask, cv.HOUGH_GRADIENT,
                                      dp=1,
                                      minDist=1000,
                                      param1=1.0,
                                      param2=10,
                                      minRadius=5,
                                      maxRadius=25)
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                log.info("Color: " + color + " circle: " + str(circles))
                # loop over the (x, y) coordinates and radius of the circles
                if len(circles) > 1:
                    log.warning("More circles for color: " + color + " found. Taking the first in the list")
                (u, v, r) = circles[0]
                # Get last position of the given marker
                marker = markers.get(color, None)
                # If no ID in the dictionary, then add one
                if marker is None:
                    marker = Marker(color, MarkerType.COLOR_BALL)
                    markers[color] = marker

                x, y, z = self.calculate_xyz_from_uv(u, v, depth_image)
                # update markers position in the dictionary
                marker.set_position(x, y, z)

                # calculate radius from uv space to xyz
                rx, ry, rz = self.calculate_xyz_from_uv(u + r, v, depth_image)
                p1 = np.array([x, y, z])
                p2 = np.array([rx, ry, rz])
                radius = np.sqrt(np.sum((p1 - p2) ** 2, axis=0))
                marker.set_radius(radius)
            else:
                log.warning("No ball found for color: " + color)

        return markers
