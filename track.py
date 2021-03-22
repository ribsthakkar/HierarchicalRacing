import math
from copy import copy
from functools import lru_cache

import numpy as np
import shapely.geometry as geom

from bezier_util import dist
from util import circ_slice


class Track():
    def __init__(self, track_center_x, track_center_y, track_width):
        self.center_x = track_center_x
        self.center_y = track_center_y
        self.width = track_width
        self.boundary1_x, self.boundary1_y = zip(*self._generate_right_boundary())
        self.boundary2_x, self.boundary2_y = zip(*self._generate_left_boundary())
        self.center_coords = np.array([*zip(self.center_x, self.center_y)])
        self.line = geom.LineString(self.center_coords)
        self.vehicles_on_track = []
        self.cars_ahead = {}
        self.cars_side = {}

    def _generate_right_boundary(self):
        for i in range(len(self.center_x)):
            if i == len(self.center_y) - 1: break
            dy = self.center_y[i+1] - self.center_y[i]
            dx = self.center_x[i+1] - self.center_x[i]
            dist = math.sqrt(dy ** 2 + dx ** 2)
            arr = np.array([dx * math.cos(90) + dy * math.sin(90), -dx*math.sin(90) + dy*math.cos(90)])
            arr = arr * (self.width/2)/dist
            yield self.center_x[i] + arr[0], self.center_y[i]+arr[1]

    def _generate_left_boundary(self):
        for i in range(len(self.center_x)):
            if i == len(self.center_y) - 1: break
            dy = self.center_y[i+1] - self.center_y[i]
            dx = self.center_x[i+1] - self.center_x[i]
            dist = math.sqrt(dy ** 2 + dx ** 2)
            arr = np.array([dx * math.cos(90) - dy * math.sin(90), dx*math.sin(90) + dy*math.cos(90)])
            arr = arr * (self.width/2)/dist
            yield self.center_x[i] + arr[0], self.center_y[i]+arr[1]

    def find_pos_index(self, init_px, currx, curry, point_horizon=400):
        min_dist = np.inf
        min_idx = 999999
        for i in range(init_px, init_px + point_horizon):
            idx = i % len(self.center_x)
            d = dist(self.center_x[idx], self.center_y[idx], currx, curry)
            if d < min_dist:
                min_dist = d
                min_idx = i
        return min_idx

    def _generate_arc_points(self, car, time_step, from_center=math.pi/12):
        # Code snippet adapted from: https://stackoverflow.com/questions/30762329/how-to-create-polygons-with-arcs-in-shapely-or-a-better-library
        centerx, centery = car.state.x, car.state.y
        radius = car.state.v * time_step

        start_angle, end_angle = car.state.heading-from_center, car.state.heading+from_center  # In degrees
        numsegments = 100

        # The coordinates of the arc
        theta = np.linspace(start_angle, end_angle, numsegments)
        x = centerx + radius * np.cos(theta)
        y = centery + radius * np.sin(theta)
        return np.column_stack([x, y])

    def _generate_heading_sweep(self, car, time_step):
        arc_points = self._generate_arc_points(car, time_step)
        poly_points = np.vstack([arc_points, [car.state.x, car.state.y]])
        sweep = geom.Polygon(poly_points)
        return sweep


    def place_car_of_type(self, car_type, x, y, dx, dy, d2x, d2y, heading, car_profile, optimizer_parameters):
        car = car_type(x, y, dx, dy, d2x, d2y, heading, car_profile, self, optimizer_parameters)
        self.vehicles_on_track.append(car)
        self.update_cars_ahead_side()
        return car

    @lru_cache(maxsize=500)
    def distance_to_center(self, x, y):
        point = geom.Point(x, y)
        return point.distance(self.line)

    @lru_cache(maxsize=500)
    def distance_to_center_custom_range(self, x, y, min_pt_hz, max_pt_hz):
        line = geom.LineString(circ_slice(self.center_coords, min_pt_hz, max_pt_hz-min_pt_hz))
        point = geom.Point(x, y)
        return point.distance(line)

    # @staticmethod
    # def generate_track():
    #
    def get_car_ordering(self):
        # Sort cars by tpx, velocity, and create data structures saying cars_ahead[car] car_behind[car] and car_side[car]
        ordering = list(sorted(self.vehicles_on_track, key=lambda car: (car.state.tpx, car.state.v), reverse=True))
        return ordering

    def update_cars_ahead_side(self, time_step=0.5):
        ordering = self.get_car_ordering()
        for i in range(len(ordering) - 1, -1, -1):
            car = ordering[i]
            sweep = self._generate_heading_sweep(car, time_step)
            self.cars_ahead[car] = []
            self.cars_side[car] = []
            for j in range(i):
                if ordering[j].state.tpx == car.state.tpx:
                    self.cars_side[car].append(copy(ordering[j].state))
                point = geom.Point(ordering[j].state.x, ordering[j].state.y)
                if sweep.contains(point):
                    self.cars_side[car].append(copy(ordering[j].state))
        print(self.cars_ahead, self.cars_side)