import math

from car_modes import DriveModes
from util import dist


class CarState:
    def __init__(self, x, y, dx, dy, d2x, d2y, track, heading):
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.d2x = d2x
        self.d2y = d2y
        self.v = math.sqrt(self.dx**2 + self.dy**2)
        self.heading = heading
        self.beta = 0
        self.tpx = track.find_pos_index(0, x, y)
        self.mode = None

    def update(self, acceleration, steering_angle, mode, lr, lf, time_step, track, **kwargs):
        raise NotImplementedError()


class FourModeCarState(CarState):
    def __init__(self, x, y, dx, dy, d2x, d2y, track, heading):
        super().__init__(x, y, dx, dy, d2x, d2y, track, heading)
        self.mode_updates = {DriveModes.FOLLOW: self._update_follow_mode,
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
        return self._race_mode_acc_contol(acceleration, steering_angle, mode, time_step, track, kwargs)

    def _race_mode_acc_contol(self, acceleration, steering_angle, mode, time_step, track, kwargs):
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



class InputModeCarState(CarState):
    def __init__(self, x, y, dx, dy, d2x, d2y, track, heading):
        super().__init__(x, y, dx, dy, d2x, d2y, track, heading)

    def update(self, acceleration, steering_angle, mode, lr, lf, time_step, track, **kwargs):
        """
        Following Kinematic Bicycle model found: https://archit-rstg.medium.com/two-to-four-bicycle-model-for-car-898063e87074
        """
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
        self.v = self.v + acceleration * time_step
        self.mode = mode
        self.tpx = track.find_pos_index(self.tpx, self.x, self.y)
