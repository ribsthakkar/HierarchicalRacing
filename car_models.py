import math
from car_modes import DriveModes
from car_states import InputModeCarState, FourModeCarState
from util import dist

class Car:
    def __init__(self, car_profile, track, controller_parameters):
        self.max_vel = car_profile['max_velocity']
        self.acc_profile = car_profile['acceleration_profile']
        self.width = car_profile['car_width']
        self.length = car_profile['car_length']
        self.max_gs = car_profile['max_cornering_gs']
        self.max_steering_angle = car_profile['max_steering_angle']
        self.max_acceleration = car_profile['max_acceleration']
        self.max_braking = -car_profile['max_braking']
        self.control_type = controller_parameters['control_type']
        self.track = track
        self.control_params = controller_parameters
        self.state = None

    def input_mode_command(self, *args, **kwargs):
        raise NotImplementedError()

    def input_steer_accelerate_command(self, *args, **kwargs):
        raise NotImplementedError()

    def plan_optimal_trajectory(self, all_cars, replan_time, input_update_time):
        optimizer = self.control_params['optimizer']
        other_cars = list(filter(lambda c: c != self, all_cars))
        return optimizer(self, other_cars, replan_time, input_update_time)

    def get_control_type(self):
        return self.control_type


class FourModeCar(Car):
    def __init__(self, x, y, dx, dy, d2x, d2y, heading, car_profile, track, controller_parameters):
        super().__init__(car_profile, track, controller_parameters)
        self.state = FourModeCarState(x, y, dx, dy, d2x, d2y, heading, track)

    def input_steer_accelerate_command(self, acceleration, steering_angle, mode, time_step):
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
    def __init__(self, x, y, dx, dy, d2x, d2y, heading, car_profile, track, controller_parameters):
        super().__init__(car_profile, track, controller_parameters)
        self.state = InputModeCarState(x, y, dx, dy, d2x, d2y, heading, track, self.max_acceleration, self.max_braking,
                                       self.max_gs, self.max_vel, self.max_steering_angle)

    def input_steer_accelerate_command(self, acceleration, steering_angle, mode, time_step):
        if steering_angle < -math.radians(self.max_steering_angle):
            steering_angle = -math.radians(self.max_steering_angle)
        if steering_angle > math.radians(self.max_steering_angle):
            steering_angle = math.radians(self.max_steering_angle)
        if acceleration > self.max_acceleration:
            acceleration = self.max_acceleration
        if acceleration < self.max_braking:
            acceleration = self.max_braking
        target_v = self.state.v + acceleration * time_step
        target_heading = self.state.heading + (target_v * math.tan(steering_angle) * math.cos(self.state.side_slip) / (self.length)) * time_step
        input_mode = (target_v, target_heading)
        acceleration, steering_angle, mode = self.state.update(input_mode, self.length / 2, self.length / 2, time_step, self.track)
        return acceleration, steering_angle, mode

    def input_mode_command(self, mode, time_step):
        if mode == None:
            return self.state.update(self.state.mode, self.length / 2, self.length / 2, time_step, self.track)
        else:
            return self.state.update(mode, self.length / 2, self.length / 2, time_step, self.track)