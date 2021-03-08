import math
from collections import deque
from enum import Enum

import shapely.geometry as geom
import numpy as np

from util import dist


class DriveModes(Enum):
    FOLLOW = 1
    PASS = 2
    RACE = 3
    BLOCK = 4

class CarState:
    def __init__(self, x, y, dx, dy, d2x, d2y, tpx, heading, mode=DriveModes.RACE):
        self.tpx = tpx
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.d2x = d2x
        self.d2y = d2y
        self.v = math.sqrt(self.dx**2 + self.dy**2)
        self.heading = heading
        self.beta = 0
        self.mode = mode
        self.mode_updates = {DriveModes.FOLLOW:self._update_follow_mode,
                             DriveModes.PASS: self._update_pass_mode,
                             DriveModes.RACE: self._update_race_mode,
                             DriveModes.BLOCK: self._update_pass_mode}

    def _update_race_mode(self, acceleration, steering_angle, mode, lr, lf, time_step, track, **kwargs):
        new_dx = self.v * math.cos(self.beta + self.heading)
        new_dy = self.v * math.sin(self.beta + self.heading)
        self.x = self.x + new_dx * time_step
        self.y = self.y + new_dy * time_step
        self.d2x = (new_dx - self.dx) / time_step
        self.d2y = (new_dy - self.dy) / time_step
        self.dx = new_dx
        self.dy = new_dy
        self.heading = self.heading + (self.v * math.sin(self.beta) / lr) * time_step
        self.beta = math.atan((lr * steering_angle) / (lr + lf))
        self.mode = mode
        self.tpx = track.find_pos_index(self.tpx, self.x, self.y)
        new_v = self.v + acceleration * time_step
        # Iteratively reduce true acceleration input until no more collisions
        # If no acceleration input is feasible then collision is unavoidable and return max braking
        # (it is inexact and somewhat inefficient, but it suffices for simulation)
        if new_v < 0:
            new_v = 0
        while True:
            collision = False
            new_dx = new_v * math.cos(self.beta + self.heading)
            new_dy = new_v * math.sin(self.beta + self.heading)
            new_x = self.x + new_dx * time_step
            new_y = self.y + new_dy * time_step
            for other_car in track.vehicles_on_track:
                if other_car.state is self: continue
                s = other_car.state
                oc_dx = s.v * math.cos(s.beta + s.heading)
                oc_dy = s.v * math.sin(s.beta + s.heading)
                oc_x = s.x + oc_dx * time_step
                oc_y = s.y + oc_dy * time_step
                if dist(new_x, new_y, oc_x, oc_y) < 2:
                    acceleration = max(kwargs['max_braking'], acceleration - 0.05)
                    new_v = self.v + acceleration * time_step
                    if new_v < 0:
                        self.v = 0
                        return acceleration, steering_angle, mode
                    collision = True
                    break
            if not collision or acceleration == kwargs['max_braking']:
                self.v = new_v
                return acceleration, steering_angle, mode

    def _update_pass_mode(self, acceleration, steering_angle, mode, lr, lf, time_step, track, **kwargs):
        pass
    def _update_block_mode(self, acceleration, steering_angle, mode, lr, lf, time_step, track, **kwargs):
        pass
    def _update_follow_mode(self, acceleration, steering_angle, mode, lr, lf, time_step, track, **kwargs):
        pass

    def update(self, acceleration, steering_angle, mode, lr, lf, time_step, track, **kwargs):
        """
        Following Kinematic Bicycle model found: https://archit-rstg.medium.com/two-to-four-bicycle-model-for-car-898063e87074
        """
        return self.mode_updates[mode](acceleration, steering_angle, mode, lr, lf, time_step, track, **kwargs)


class Car:
    def __init__(self, car_profile, car_state, track, optimizer_parameters):
        self.max_vel = car_profile['max_velocity']
        self.acc_profile = car_profile['acceleration_profile']
        self.width = car_profile['car_width']
        self.length = car_profile['car_length']
        self.max_gs = car_profile['max_cornering_gs']
        self.max_steering_angle = car_profile['max_steering_angle']
        self.max_acceleration = car_profile['max_acceleration']
        self.max_braking = -car_profile['max_braking']
        self.state = car_state
        self.track = track
        self.opt_params = optimizer_parameters

    def update_state(self, acceleration, steering_angle, mode, time_step):
        if steering_angle < -math.radians(self.max_steering_angle):
            steering_angle = -math.radians(self.max_steering_angle)
        if steering_angle > math.radians(self.max_steering_angle):
            steering_angle = math.radians(self.max_steering_angle)
        if acceleration > self.max_acceleration:
            acceleration = self.max_acceleration
        if acceleration < self.max_braking:
            acceleration = self.max_braking
        if self.state.v >= self.max_vel:
            acceleration = 0
        acceleration, steering_angle, mode = self.state.update(acceleration, steering_angle, mode, self.length / 2,
                                                               self.length / 2, time_step, self.track,
                                                               max_braking=self.max_braking)
        return acceleration, steering_angle, mode

    def plan_optimal_trajectory(self, all_cars, replan_time, input_update_time):
        optimizer = self.opt_params['optimizer']
        other_cars = list(filter(lambda c: c != self, all_cars))
        return optimizer(self, other_cars, replan_time, input_update_time)
