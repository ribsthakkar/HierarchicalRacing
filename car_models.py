import math
from car_modes import DriveModes
from car_states import InputModeCarState, FourModeCarState
from util import dist

class Car:
    def __init__(self, car_profile, track, optimizer_parameters):
        self.max_vel = car_profile['max_velocity']
        self.acc_profile = car_profile['acceleration_profile']
        self.width = car_profile['car_width']
        self.length = car_profile['car_length']
        self.max_gs = car_profile['max_cornering_gs']
        self.max_steering_angle = car_profile['max_steering_angle']
        self.max_acceleration = car_profile['max_acceleration']
        self.max_braking = -car_profile['max_braking']
        self.track = track
        self.opt_params = optimizer_parameters
        self.state = None

    def input_command(self, *args):
        raise NotImplementedError()

    def plan_optimal_trajectory(self, all_cars, replan_time, input_update_time):
        optimizer = self.opt_params['optimizer']
        other_cars = list(filter(lambda c: c != self, all_cars))
        return optimizer(self, other_cars, replan_time, input_update_time)


class FourModeCar(Car):
    def __init__(self, car_profile, car_state, track, optimizer_parameters):
        super().__init__(car_profile, track, optimizer_parameters)
        self.state = car_state

    def input_command(self, acceleration, steering_angle, mode, time_step):
        if steering_angle < -math.radians(self.max_steering_angle):
            steering_angle = -math.radians(self.max_steering_angle)
        if steering_angle > math.radians(self.max_steering_angle):
            steering_angle = math.radians(self.max_steering_angle)
        if acceleration > self.max_acceleration:
            acceleration = self.max_acceleration
        if acceleration < self.max_braking:
            acceleration = self.max_braking
        acceleration, steering_angle, mode = self.state.update(acceleration, steering_angle, mode, self.length / 2,
                                                               self.length / 2, time_step, self.track,
                                                               max_braking=self.max_braking)
        return acceleration, steering_angle, mode


class DiscreteInputModeCar(Car):
    def __init__(self, car_profile, car_state, track, optimizer_parameters):
        super().__init__(car_profile, track, optimizer_parameters)
        self.state = car_state

    def input_command(self, acceleration, steering_angle, mode, time_step):
        return self.state.update(acceleration, steering_angle, mode, self.length / 2,
                                                               self.length / 2, time_step, self.track)


def generate_car(car_type, x, y, dx, dy, d2x, d2y, heading, car_profile, track, optimizer_parameters):
    car_state_types = {DiscreteInputModeCar: InputModeCarState, FourModeCar: FourModeCarState}
    car_state = car_state_types[car_type](x, y, dx, dy, d2x, d2y, track, heading)
    return car_type(car_profile, car_state, track, optimizer_parameters)