import math

from car_modes import DiscreteVelocityHeadingModeManager
from util import dist, find_smallest_rotation, TPI


class CarState:
    def __init__(self, x, y, dx, dy, d2x, d2y, heading, length, width, track=None):
        self.x = x
        self.y = y
        self.old_x = x
        self.old_y = y

        self.dx = dx
        self.dy = dy
        self.d2x = d2x
        self.d2y = d2y
        self.v = math.sqrt(self.dx**2 + self.dy**2)
        self.dv = math.sqrt(self.d2x**2+self.d2y**2)

        self.heading = heading
        self.dheading = 0

        self.side_slip = 0

        self.l = length
        self.w = width
        if track: self.tpx = track.find_pos_index(0, x, y)

    def update(self, acceleration, steering_angle, lr, lf, time_step, track, **kwargs):
        raise NotImplementedError()


class BicycleCarState(CarState):
    def __init__(self, x, y, dx, dy, d2x, d2y, length, width, heading, track):
        super().__init__(x, y, dx, dy, d2x, d2y, heading, length, width, track)

    def update(self, acceleration, steering_angle, lr, lf, time_step, track, **kwargs):
        """
        Following Kinematic Bicycle model found: https://archit-rstg.medium.com/two-to-four-bicycle-model-for-car-898063e87074
        """
        new_dx = self.v * math.cos(self.side_slip + self.heading)
        new_dy = self.v * math.sin(self.side_slip + self.heading)
        self.dheading = self.v * math.sin(self.side_slip)/lr
        self.dv = acceleration
        
        self.x = self.x + new_dx * time_step
        self.y = self.y + new_dy * time_step
        self.heading = self.heading + self.dheading * time_step
        self.v = self.v + self.dv * time_step

        self.d2x = (new_dx - self.dx) / time_step
        self.d2y = (new_dy - self.dy) / time_step
        self.dx = new_dx
        self.dy = new_dy

        self.side_slip = math.atan((lr * math.tan(steering_angle)) / (lr + lf))
        self.tpx = track.find_pos_index(self.tpx, self.x, self.y)
        return acceleration, steering_angle, None


class DiscreteVelocityHeadingInputCarState(CarState):
    def __init__(self, x, y, dx, dy, d2x, d2y, heading,  track, length, width, max_acceleration, max_braking, max_cornering_gs, max_velocity, max_steering_angle, manager_params=None):
        super().__init__(x, y, dx, dy, d2x, d2y, heading, length, width, track)
        if manager_params:
            self.mode_manager = DiscreteVelocityHeadingModeManager(max_acceleration, max_braking, max_cornering_gs, max_velocity, max_steering_angle, manager_params['v_prec'], manager_params['h_prec'])
        else:
            self.mode_manager = DiscreteVelocityHeadingModeManager(max_acceleration, max_braking, max_cornering_gs, max_velocity, max_steering_angle)
        self.mode = self.mode_manager.mode_from_velocity_heading(self.v, heading)
        self.last_mode = self.mode
        self.v = self.mode[0]
        self.heading = self.mode[1]


    def update(self, mode, lr, lf, time_step, track, **kwargs):
        """
        Following Kinematic Bicycle model found: https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
        in part 2.3 with some modifications to use average velocity when computing steering angle and average velocity to
        compute position.
        """
        self.old_v = self.v
        self.old_heading = self.heading
        self.last_mode = self.mode
        new_dx = self.v * math.cos(self.side_slip + self.heading)
        new_dy = self.v * math.sin(self.side_slip + self.heading)

        self.v, self.heading, self.mode = self.mode_manager.try_switch(self, mode[0], mode[1], time_step, cars_ahead=kwargs['cars_ahead'], cars_side=kwargs['cars_side'])
        self.dv = (self.v - self.old_v)/time_step
        self.dheading = find_smallest_rotation(self.heading, self.old_heading)/time_step if not math.isclose(self.heading - self.old_heading, 0, abs_tol=1e-6) else 0
        steering_angle = math.atan((self.dheading * (lr + lf))/(self.old_v*math.cos(self.side_slip)))
        self.x = self.x + (new_dx) * time_step * 0.5
        self.y = self.y + (new_dy + self.dy) * time_step * 0.5
        self.d2x = (new_dx - self.dx) / time_step
        self.d2y = (new_dy - self.dy) / time_step
        self.dx = new_dx
        self.dy = new_dy

        self.side_slip = math.atan((lr * math.tan(steering_angle)/(lr + lf)))
        self.tpx = track.find_pos_index(self.tpx, self.x, self.y)
        return self.dv, steering_angle, self.mode
