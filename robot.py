import numpy as np
import math


class Robot:

    def __init__(self):
        self.pos = (None, None)  # Tuple of x and y
        self.lidar_center = (None, None)  # Tuple of x and y
        self.theta = None
        self.lidar_dists = [None] * 5
        self.lidar_points = [None] * 5
        self.lidar_point_is_wall = [None] * 5

    def update_robot(self, pos, theta_deg, lidar_dists):
        self.pos = pos
        self.theta = math.radians(theta_deg)
        self.lidar_dists = lidar_dists
        self.calculate_lidar_dists()
        # update data map

    def calculate_lidar_dists(self):

        c, s = np.cos(self.theta), np.sin(self.theta)
        r_mat = np.array(((c, -s), (s, c)))

        c_pos_45, s_pos_45 = math.cos(math.radians(45)), math.sin(math.radians(45))
        c_neg_45, s_neg_45 = math.cos(math.radians(-45)), math.sin(math.radians(-45))

        self.lidar_center = self.pos + r_mat.dot(np.array((0.08, 0)))

        self.lidar_point_is_wall = []
        for idx in range(len(self.lidar_dists)):
            if self.lidar_dists[idx] < 0:  # No point detected by the lidar
                self.lidar_dists[idx] = 0.2
                self.lidar_point_is_wall.append(False)
            else:
                self.lidar_point_is_wall.append(True)

        self.lidar_points = []

        lidar_p0 = np.array(self.pos) + r_mat.dot(np.array((0.08, self.lidar_dists[0])))
        self.lidar_points.append(tuple(lidar_p0))

        lidar_p1 = np.array(self.pos) + r_mat.dot(np.array((0.08 + c_pos_45 * self.lidar_dists[1], s_pos_45 * self.lidar_dists[1])))
        self.lidar_points.append(tuple(lidar_p1))

        lidar_p2 = np.array(self.pos) + r_mat.dot(np.array((0.08 + self.lidar_dists[2], 0)))
        self.lidar_points.append(tuple(lidar_p2))

        lidar_p3 = np.array(self.pos) + r_mat.dot(np.array((0.08 + c_neg_45 * self.lidar_dists[3], s_neg_45 * self.lidar_dists[3])))
        self.lidar_points.append(tuple(lidar_p3))

        lidar_p4 = np.array(self.pos) + r_mat.dot(np.array((0.08, - self.lidar_dists[4])))
        self.lidar_points.append(tuple(lidar_p4))

    # Increase coordinates in direction of detection if an object is detected, decrease otherwise
    def add_tolerance_to_lidar_points(self, tolerance):
        for idx in range(len(self.lidar_points)):
            direction_detection = np.subtract(self.lidar_points[idx], self.lidar_center)
            direction_detection = direction_detection / np.linalg.norm(direction_detection)

            if self.lidar_point_is_wall[idx]:
                self.lidar_points[idx] = tuple(np.array(self.lidar_points[idx]) + direction_detection*tolerance)
            else:
                self.lidar_points[idx] = tuple(np.array(self.lidar_points[idx]) - direction_detection * tolerance)