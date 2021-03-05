import math
from collections import deque
from enum import Enum

import shapely.geometry as geom
import numpy as np


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

    def update(self, acceleration, steering_angle, mode, lr, lf, time_step, track):
        """
        Following Kinematic Bicycle model found: https://archit-rstg.medium.com/two-to-four-bicycle-model-for-car-898063e87074
        :param acceleration:
        :param steering_angle:
        :param mode:
        :param lr:
        :param lf:
        :param time_step:
        :return:
        """
        new_dx = self.v * math.cos(self.beta + self.heading)
        new_dy = self.v * math.sin(self.beta + self.heading)
        self.x = self.x + new_dx * time_step
        self.y = self.y + new_dy * time_step
        self.d2x = (new_dx - self.dx)/time_step
        self.d2y = (new_dy - self.dy)/time_step
        self.dx = new_dx
        self.dy = new_dy
        self.heading = self.heading + (self.v * math.sin(self.beta)/lr) * time_step
        self.beta = math.atan((lr * steering_angle) / (lr + lf))
        self.v = self.v + acceleration * time_step
        self.mode = mode
        self.tpx = track.find_pos_index(self.tpx, self.x, self.y)
        print(math.degrees(self.heading), math.degrees(self.beta))

class Car:
    def __init__(self, car_profile, car_state, track, optimizer_parameters):
        self.max_vel = car_profile['max_velocity']
        self.acc_profile = car_profile['acceleration_profile']
        self.width = car_profile['car_width']
        self.length = car_profile['car_length']
        self.max_gs = car_profile['max_cornering_gs']
        self.max_steering_angle = car_profile['max_steering_angle']
        self.max_acceleration = car_profile['max_acceleration']
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
        self.state.update(acceleration, steering_angle, mode, self.length / 2, self.length / 2, time_step, self.track)
        return acceleration, steering_angle, mode

    def plan_optimal_trajectory(self, all_cars, replan_time, input_update_time):
        optimizer = self.opt_params['optimizer']
        other_cars = list(filter(lambda c: c != self, all_cars))
        return optimizer(self, other_cars, replan_time, input_update_time)
