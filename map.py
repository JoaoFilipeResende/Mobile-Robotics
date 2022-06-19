import copy

import numpy as np
from heapdict import heapdict
import math

from numba import jit


class Map:
    def __init__(self, map_width, map_height):
        self.map_matrix = None  # 0/1 means free/occupied
        self.map_matrix_confidence = []  # 0/1 means 100% confidence free/occupied, 0.5 means 50% confidence free

        if np.round(0.5) != 0:
            raise Exception("Your python version does not round half to even, please use Python 3")

        for idx in range(map_width * map_height):
            self.map_matrix_confidence.append(0.5)

        self.map_matrix_confidence = np.array(self.map_matrix_confidence).reshape((map_height, map_width))
        self.map_matrix = np.round(self.map_matrix_confidence)

    def clean_map(self):
        self.map_matrix_confidence = []

        for idx in range(self.map_matrix.shape[0] * self.map_matrix.shape[1]):
            self.map_matrix_confidence.append(0.5)

        self.map_matrix_confidence = np.array(self.map_matrix_confidence).reshape(
            (self.map_matrix.shape[0], self.map_matrix.shape[1]))
        self.map_matrix = np.round(self.map_matrix_confidence)

    def update_map(self, points, points_are_occupied):

        for idx in range(len(points)):
            point = self.world_to_map_coords(points[idx])
            if (0 <= point[0] < self.map_matrix.shape[1]) and (0 <= point[1] < self.map_matrix.shape[0]):
                if points_are_occupied[idx]:
                    self.map_matrix_confidence[point[1], point[0]] += 0.1
                    if self.map_matrix_confidence[point[1], point[0]] > 1:
                        self.map_matrix_confidence[point[1], point[0]] = 1
                else:
                    self.map_matrix_confidence[point[1], point[0]] -= 0.1
                    if self.map_matrix_confidence[point[1], point[0]] < 0:
                        self.map_matrix_confidence[point[1], point[0]] = 0

        old_map = np.copy(self.map_matrix)

        self.map_matrix = np.round(self.map_matrix_confidence)

        return not np.array_equal(old_map, self.map_matrix)

    def get_map_size(self):
        return self.map_matrix.shape[0], self.map_matrix.shape[1]

    def is_map_coord_occupied(self, point):
        if self.map_matrix[point[1], point[0]] == 0:
            return False
        else:
            return True

    def shortest_path(self, start_point, end_point, post_processing=False):  # start_point is the robot
        path = self._a_star(self.world_to_map_coords(start_point), self.world_to_map_coords(end_point))
        path_simplified = None
        if post_processing:
            path_simplified = self.post_processing(self.world_to_map_coords(start_point, False), path)
        if path:
            path = [self.map_to_world_coords(point) for point in path]
            if path_simplified:
                path_simplified = [self.map_to_world_coords(point) for point in path_simplified]
                return path, path_simplified
            else:
                return path, None
        return None, None

    def post_processing(self, start_point, path):
        if not path or len(path) < 2:
            return path

        path = copy.deepcopy(path)

        path.insert(0, start_point)
        idx_a = 0

        while True:
            for idx_b in range(len(path) - 1, idx_a + 1, -1):
                if self.path_is_clear(path[idx_a], path[idx_b], 0.8):
                    path = path[:idx_a + 1] + path[idx_b:]
                    break

            idx_a += 1
            if idx_a >= len(path) - 2:
                break
        path.pop(0)
        return path

    # A path is considered clear if 3 parallel line segments separated by "width" do not intersect any obstacles
    # Points A and B are the start and end points of the middle-most line

    def path_is_clear(self, point_a, point_b, width):

        # Create a vector that is perpendicular to line AB
        theta = math.atan2(point_b[1] - point_a[1], point_b[0] - point_a[0])
        c, s = np.cos(theta + math.radians(90)), np.sin(theta + math.radians(90))
        r_mat = np.array(((c, -s), (s, c)))
        vector_perpendicular = r_mat.dot(np.array([width / 2, 0]))

        # Create line segment CD that is parallel to AB
        point_c = np.array(point_a) + vector_perpendicular
        point_d = np.array(point_b) + vector_perpendicular

        # Create line segment EF that is parallel to AB
        point_e = np.array(point_a) - vector_perpendicular
        point_f = np.array(point_b) - vector_perpendicular

        if self.line_segment_intersects_obstacles(point_a, point_b) or \
                self.line_segment_intersects_obstacles(point_c, point_d) or \
                self.line_segment_intersects_obstacles(point_e, point_f):
            return False
        else:
            return True

    def line_segment_intersects_obstacles(self, point_a, point_b):
        for x in range(self.map_matrix.shape[1]):
            for y in range(self.map_matrix.shape[0]):
                if self.is_map_coord_occupied((x, y)):
                    if self.aabb_contains_segment(point_a, point_b, (x, y)):
                        return True
        return False

    @staticmethod
    @jit(nopython=True)
    def aabb_contains_segment(point_a, point_b, aabb_center):

        # Limits of the AABB
        aabb_min_x = aabb_center[0] - 0.5
        aabb_max_x = aabb_center[0] + 0.5
        aabb_min_y = aabb_center[1] - 0.5
        aabb_max_y = aabb_center[1] + 0.5

        # Check if line segment is completely outside AABB
        if ((point_a[0] < aabb_min_x) and (point_b[0] < aabb_min_x)) or \
                ((point_a[1] < aabb_min_y) and (point_b[1] < aabb_min_y)) or \
                ((point_a[0] > aabb_max_x) and (point_b[0] > aabb_max_x)) or \
                ((point_a[1] > aabb_max_y) and (point_b[1] > aabb_max_y)):
            return False

        m = (point_b[1] - point_a[1]) / (point_b[0] - point_a[0])

        # Check left side collision
        y = m * (aabb_min_x - point_a[0]) + point_a[1]
        if aabb_min_y <= y <= aabb_max_y:
            return True

        # Check right side collision
        y = m * (aabb_max_x - point_a[0]) + point_a[1]
        if aabb_min_y <= y <= aabb_max_y:
            return True

        # Check bottom side collision
        x = (aabb_min_y - point_a[1]) / m + point_a[0]
        if aabb_min_x <= x <= aabb_max_x:
            return True

        # Check top side collision
        x = (aabb_max_y - point_a[1]) / m + point_a[0]
        if aabb_min_x <= x <= aabb_max_x:
            return True

        return False

    def _a_star(self, start_point, end_point):

        closed_set = set()
        open_set = heapdict()  # Dict of tuples (g+heuristic, g)
        prev_points = {start_point: None}

        open_set[start_point] = (self.dist(start_point, end_point), 0)

        while open_set:
            point, (g_plus_heuristic, g) = open_set.popitem()
            closed_set.add(point)

            if point != end_point:
                for neighbor_delta in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                    nb = np.add(point, neighbor_delta)
                    nb = tuple(nb)
                    if (0 <= nb[0] < self.map_matrix.shape[0]) and (0 <= nb[1] < self.map_matrix.shape[1]) \
                            and (nb not in closed_set):
                        if not self.is_map_coord_occupied(nb):
                            nb_g = g + 1
                            if nb in open_set:
                                old_nb_g = open_set[nb][1]
                                if nb_g < old_nb_g:
                                    open_set[nb] = (nb_g + self.dist(nb, end_point), nb_g)
                                    prev_points[nb] = point
                            else:
                                open_set[nb] = (nb_g + self.dist(nb, end_point), nb_g)
                                prev_points[nb] = point

            else:
                path = []
                while point is not None:
                    path.append(point)
                    point = prev_points[point]
                return path[::-1]

        # If the open set has been emptied but the end point was never found, then no path exists
        return None

    @staticmethod
    def world_to_map_coords(world_coords, do_rounding=True):
        if do_rounding:
            return round(world_coords[0] / 0.25), round(15 - (world_coords[1] / 0.25))
        else:
            return world_coords[0] / 0.25, 15 - (world_coords[1] / 0.25)

    @staticmethod
    def map_to_world_coords(map_coords):
        return map_coords[0] * 0.25, (15 - map_coords[1]) * 0.25

    @staticmethod
    @jit(nopython=True)
    def dist(point_a, point_b):
        return np.sqrt((point_b[0] - point_a[0]) ** 2 + (point_b[1] - point_a[1]) ** 2)

