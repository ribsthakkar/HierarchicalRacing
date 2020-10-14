import math
from collections import deque
import shapely.geometry as geom
import numpy as np

class Car():
    def __init__(self, cof, max_vel, max_accel, time_delta=0.001):
        self.cof = cof
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.track_index = 0
        self.x = 0.0
        self.y = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.d2x = 0.0
        self.d2y = 0.0
        self.actions = deque()
        self.visited_x=[]
        self.visited_y=[]
        self.time_delta=time_delta

    def progress_time(self):
        self.visited_x.append(self.x)
        self.visited_y.append(self.y)
        self.d2x, self.d2y = self.actions.popleft()
        self.x += self.time_delta*(self.dx + 0.5 * self.d2x*self.time_delta)
        self.y += self.time_delta*(self.dy + 0.5 * self.d2y*self.time_delta)
        self.dx += self.d2x*self.time_delta
        self.dy = self.d2y*self.time_delta

    def _calculate_nearest_point_index(self):
        point = geom.Point(self.x, self.y)
        dist = point.distance(self.line)
        pass

    def place_on_track(self, initial_x, initial_y, target_trajectory_x, target_trajectory_y):
        self.x = initial_x
        self.y = initial_y
        self.main_track_x = target_trajectory_x
        self.main_track_y = target_trajectory_y
        self.main_track = np.array([*zip(self.main_track_x, self.main_track_y)])
        self.line = geom.LineString(self.main_track)

        self.N = self._calculate_nearest_point_index()

    def simulate_motion(self):
        time = 0

        while time < 100:
            if math.isclose(math.fmod(time, 0.05), 0.0):
                # Plan horizon
                self.calculate_horizon()
                pass
            else:
                self.progress_time()
            time += self.time_delta