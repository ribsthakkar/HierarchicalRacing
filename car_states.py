import math

from car_modes import DriveModes, InputModes
from util import dist, find_smallest_rotation


class CarState:
    def __init__(self, x, y, dx, dy, d2x, d2y, heading, length, width,  track):
        self.x = x
        self.y = y
        self.old_x = x
        self.old_y = y
        self.dx = dx
        self.dy = dy
        self.d2x = d2x
        self.d2y = d2y
        self.v = math.sqrt(self.dx**2 + self.dy**2)
        self.old_v = self.v
        self.heading = heading
        self.old_heading = self.heading
        self.side_slip = 0
        if track: self.tpx = track.find_pos_index(0, x, y)
        self.mode = None
        self.last_mode = None
        self.l = length
        self.w = width

    def update(self, acceleration, steering_angle, mode, lr, lf, time_step, track, **kwargs):
        raise NotImplementedError()


class FourModeCarState(CarState):
    def __init__(self, x, y, dx, dy, d2x, d2y, length, width, heading, track):
        super().__init__(x, y, dx, dy, d2x, d2y, heading, length, width, track)
        self.mode_updates = {DriveModes.FOLLOW: self._update_follow_mode,
                             DriveModes.PASS: self._update_pass_mode,
                             DriveModes.RACE: self._update_race_mode,
                             DriveModes.BLOCK: self._update_pass_mode}

    def _update_race_mode(self, acceleration, steering_angle, mode, lr, lf, time_step, track, **kwargs):
        new_dx = self.v * math.cos(self.side_slip + self.heading)
        new_dy = self.v * math.sin(self.side_slip + self.heading)
        self.old_x = self.x
        self.old_y = self.y
        self.x = self.x + new_dx * time_step
        self.y = self.y + new_dy * time_step
        self.d2x = (new_dx - self.dx) / time_step
        self.d2y = (new_dy - self.dy) / time_step
        self.dx = new_dx
        self.dy = new_dy
        self.old_heading = self.heading
        self.heading = self.heading + (self.v * math.sin(self.side_slip) / lr) * time_step
        self.side_slip = math.atan((lr * math.tan(steering_angle)) / (lr + lf))
        self.last_mode = self.mode
        self.mode = mode
        self.tpx = track.find_pos_index(self.tpx, self.x, self.y)
        return self._race_mode_acc_contol(acceleration, steering_angle, mode, time_step, track, **kwargs)

    def _race_mode_acc_contol(self, acceleration, steering_angle, mode, time_step, track, **kwargs):
        new_v = self.v + acceleration * time_step
        # Iteratively reduce true acceleration input until no more collisions
        # If no acceleration input is feasible then collision is unavoidable and return max braking
        # (it is inexact and somewhat inefficient, but it suffices for simulation)
        if new_v < 0:
            new_v = 0
        while True:
            collision = False
            new_dx = new_v * math.cos(self.side_slip + self.heading)
            new_dy = new_v * math.sin(self.side_slip + self.heading)
            new_x = self.x + new_dx * time_step
            new_y = self.y + new_dy * time_step
            for other_car in track.vehicles_on_track:
                if other_car.state is self: continue
                s = other_car.state
                oc_dx = s.v * math.cos(self.side_slip + self.heading)
                oc_dy = s.v * math.sin(self.side_slip + self.heading)
                oc_x = s.x + oc_dx * time_step
                oc_y = s.y + oc_dy * time_step
                if dist(new_x, new_y, oc_x, oc_y) < 2:
                    acceleration = max(kwargs['max_braking'], acceleration - 0.05)
                    new_v = self.v + acceleration * time_step
                    if new_v < 0:
                        self.old_v = self.v
                        self.v = 0
                        return acceleration, steering_angle, mode
                    collision = True
                    break
            if not collision or acceleration == kwargs['max_braking']:
                self.old_v = self.v
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



class InputModeCarState(CarState):
    def __init__(self, x, y, dx, dy, d2x, d2y, heading,  track, length, width, max_acceleration, max_braking, max_cornering_gs, max_velocity, max_steering_angle, manager_params=None):
        super().__init__(x, y, dx, dy, d2x, d2y, heading, length, width, track)
        if manager_params:
            self.mode_manager = InputModes(max_acceleration, max_braking, max_cornering_gs, max_velocity, max_steering_angle, manager_params['v_prec'], manager_params['h_prec'])
        else:
            self.mode_manager = InputModes(max_acceleration, max_braking, max_cornering_gs, max_velocity, max_steering_angle)
        self.mode = self.mode_manager.mode_from_velocity_heading(self.v, heading)
        self.last_mode = self.mode
        self.v = self.mode[0]
        self.old_v = self.v
        self.heading = self.mode[1]
        self.old_heading = self.heading


    def update(self, mode, lr, lf, time_step, track, **kwargs):
        """
        Following Kinematic Bicycle model found: https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
        in part 2.3 with some modifications to use average velocity when computing steering angle and average velocity to
        compute position.
        """
        self.old_heading = self.heading
        self.old_v = self.v
        self.last_mode = self.mode
        self.v, self.heading, self.mode = self.mode_manager.try_switch(self, mode[0], mode[1], time_step, cars_ahead=kwargs['cars_ahead'], cars_side=kwargs['cars_side'])
        acceleration = (self.v - self.old_v)/time_step
        dh = find_smallest_rotation(self.heading, self.old_heading)/time_step if not math.isclose(self.heading - self.old_heading, 0, abs_tol=1e-6) else 0
        avg_v = (self.v + self.old_v)/2
        steering_angle = math.atan(dh * (lr + lf)/(avg_v * math.cos(self.side_slip)))
        new_dx = self.v * math.cos(self.heading + self.side_slip)
        new_dy = self.v * math.sin(self.heading + self.side_slip)
        self.old_x = self.x
        self.old_y = self.y
        self.x = self.x + (new_dx + self.dx) * time_step * 0.5
        self.y = self.y + (new_dy + self.dy) * time_step * 0.5
        self.d2x = (new_dx - self.dx) / time_step
        self.d2y = (new_dy - self.dy) / time_step
        self.dx = new_dx
        self.dy = new_dy
        self.side_slip = math.atan((lr * math.tan(steering_angle)/(lr + lf)))
        self.tpx = track.find_pos_index(self.tpx, self.x, self.y)
        return acceleration, steering_angle, self.mode
