import itertools
import math
from enum import Enum
import numpy as np
from scipy import optimize
import scipy.stats as stats
import shapely.geometry as geom

from util import round_to_fraction, gravitational_acceleration

TPI = 2 * math.pi

class DriveModes(Enum):
    FOLLOW = 1
    PASS = 2
    RACE = 3
    BLOCK = 4


class InputModes:
    def __init__(self, max_acceleration, max_braking, max_cornering_gs, max_velocity, max_steering_angle, vel_precision=0.5, heading_precision= TPI/1000):
        self.max_acc = max_acceleration
        self.max_brak = max_braking
        self.max_corn = max_cornering_gs * gravitational_acceleration
        self.max_v = max_velocity
        self.max_steer = max_steering_angle
        self.total_modes = (int(max_velocity/vel_precision) + 1) * (int(TPI/heading_precision) + 1)
        self.v_prec = vel_precision
        self.h_prec = heading_precision

    def mode_from_velocity_heading(self, velocity, heading):
        v = round_to_fraction(velocity, self.v_prec)
        v = max(min(v, self.max_v), self.v_prec)
        h = round_to_fraction(heading, self.h_prec)
        # h = max(min(h, TPI), 0)
        return (v, h)

    def _find_max_cornering_acc(self, mode, targetv, targeth, dt):
        initialv, initialh = mode
        vxi = initialv * math.cos(initialh)
        vyi = initialv * math.sin(initialh)
        vxf = targetv * math.cos(targeth)
        vyf = targetv * math.sin(targeth)
        ax = (vxf - vxi) / dt
        ay = (vyf - vyi) / dt
        a = math.sqrt((ax ** 2) + (ay ** 2))

        def opt_n(t):
            a_t = 0.5 * (2 * ax * (vxi + ax * t) + 2 * ay * (vyi + ay * t)) / (
                math.sqrt((vxi + ax * t) ** 2 + (vyi + ay * t) ** 2))
            if math.isclose(abs(a), abs(a_t)) or math.isnan(a_t) or math.isinf(a_t):
                return 0
            a_n = math.sqrt((a ** 2) - (a_t) ** 2)
            return -(a_n ** 2)

        result = optimize.minimize_scalar(opt_n, bounds=(0, dt), method='bounded')
        return math.sqrt(-result.fun)

    def _find_max_longitudnal_acc(self, mode, targetv, targeth, dt):
        initialv, initialh = mode
        vxi = initialv * math.cos(initialh)
        vyi = initialv * math.sin(initialh)
        vxf = targetv * math.cos(targeth)
        vyf = targetv * math.sin(targeth)
        ax = (vxf - vxi) / dt
        ay = (vyf - vyi) / dt

        def opt_t(t):
            a_t = 0.5 * (2 * ax * (vxi + ax * t) + 2 * ay * (vyi + ay * t)) / (
                math.sqrt((vxi + ax * t) ** 2 + (vyi + ay * t) ** 2))
            if math.isnan(a_t) or math.isinf(a_t):
                return 0
            return -(a_t ** 2)

        result = optimize.minimize_scalar(opt_t, bounds=(0, dt), method='bounded')
        return math.sqrt(-result.fun)

    def _find_best_heading(self, car_state, targetv, targeth, other_trajectories, dt):
        mode = car_state.mode
        if math.isclose(targeth, mode[1]): return mode[1]
        max_bounds = self._find_smallest_rotation(targeth, mode[1])
        if self._is_cw(mode[1], targeth): bounds=(-max_bounds, 0)
        else: bounds=(0, max_bounds)
        def opt_h(h):
            d = mode[1] + h
            if self._find_max_cornering_acc(mode, targetv, d, dt) <= self.max_corn and \
                    self._no_collisions_with_cars(car_state, targetv, targeth, other_trajectories, dt):
                return -h
            return 1000000
        result = optimize.minimize_scalar(opt_h, bounds=bounds, method='bounded')
        return -result.x + mode[1] if result.x < 1000000 else mode[1]

    def _find_best_braking(self, car_state, targetv, targeth, other_trajectories, dt):
        mode = car_state.mode
        if math.isclose(targetv, mode[0]): return mode[0]
        def opt_b(v):
            if self._find_max_longitudnal_acc(mode, v, targeth, dt) <= abs(self.max_brak) and \
                    self._no_collisions_with_cars(car_state, targetv, targeth, other_trajectories, dt):
                return v
            return 1000000
        result = optimize.minimize_scalar(opt_b, bounds=(targetv, mode[0]), method='bounded')
        return result.x if result.x < 1000000 else mode[0]

    def _find_best_acc(self, car_state, targetv, targeth, other_trajectories, dt):
        mode = car_state.mode
        if math.isclose(targetv, mode[0]): return mode[0]
        def opt_b(v):
            if self._find_max_longitudnal_acc(mode, v, targeth, dt) <= self.max_acc and \
                    self._no_collisions_with_cars(car_state, targetv, targeth, other_trajectories, dt):
                return -v
            return 10000000
        result = optimize.minimize_scalar(opt_b, bounds=(mode[0], targetv), method='bounded')
        return -result.x if result.x < 1000000 else mode[0]

    def _is_cw(self, current, target):
        return math.sin(current - target) > 0

    def _find_smallest_rotation(self, target_h, current_h):
        if self._is_cw(current_h, target_h):
            rotation = current_h - target_h if current_h > target_h else current_h + TPI - target_h
            return -rotation
        else:
            rotation = target_h - current_h if target_h > current_h else target_h + TPI - current_h
            return rotation

    def _pos_functions(self, xi, yi, vi, vf, hi, hf, dt):
        vxi = vi * math.cos(hi)
        vyi = vi * math.sin(hi)
        vxf = vf * math.cos(hf)
        vyf = vf * math.sin(hf)
        ax = (vxf - vxi) / dt
        ay = (vyf - vyi) / dt
        x_pos = lambda t: xi+vxi*t+0.5*ax*t**2
        y_pos = lambda t: yi+ vyi*t+0.5*ay*t**2
        h_pos = lambda t: math.atan((vyi + ay*t)/(vxi + ax*t))
        return x_pos,y_pos, h_pos

    def _rect_from_center(self, x, y, l, w, rot):
        # Adapted from stackoverflow here: https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
        center = np.array([x, y])
        v1 = np.array([math.cos(rot), math.sin(rot)])
        v2 = np.array(-v1[1], v1[0])  # rotate by 90
        v1 *= l/2
        v2 *= w/2
        points = np.vstack([center + v1 + v2,
                            center - v1 + v2,
                            center - v1 - v2,
                            center + v1 - v2,])
        return geom.Polygon(points)

    def _estimate_position_rectangles(self, state, dt):
        vi = state.v
        hi = state.heading
        hf = math.atan((state.dy + state.d2y*dt)/(state.dx + state.d2x*dt))
        vf = math.sqrt((state.dy + state.d2y*dt)**2 + (state.dx + state.d2x*dt)**2)
        x_pos_f, y_pos_f, h_pos_f = self._pos_functions(state.x, state.y, vi, vf, hi, hf, dt)
        return [self._rect_from_center(x, y, state.l, state.w, h) for x, y, h in zip(map(x_pos_f, np.linspace(0, dt)),
                                                                                     map(y_pos_f, np.linspace(0, dt)),
                                                                                     map(h_pos_f, np.linspace(0, dt)))]

    def _generate_position_rectangles(self, state, targetv, targeth, dt):
        x_pos_f, y_pos_f, h_pos_f = self._pos_functions(state.x, state.y, state.v, targetv, state.heading, targeth, dt)
        return [self._rect_from_center(x, y, state.l, state.w, h) for x, y, h in zip(map(x_pos_f, np.linspace(0, dt)),
                                                                                     map(y_pos_f, np.linspace(0, dt)),
                                                                                     map(h_pos_f, np.linspace(0, dt)))]

    def _no_collisions_with_cars(self, car_state, targetv, targeth, other_trajectories, dt):
        # to avoid collisions with cars and trajectories we need to make sure bounding boxes don't intersect as traveling
        new_rectangles = self._generate_position_rectangles(car_state, targetv, targeth, dt)
        if not len(other_trajectories): return True
        return not any(ours.intersects(theirs) for c in other_trajectories for ours, theirs in zip(new_rectangles, other_trajectories[c]))

    def try_switch(self, car_state, velocity, heading, time_step, cars_ahead=None, cars_side=None):
        other_trajectories = {}
        mode = car_state.mode
        if cars_ahead and len(cars_ahead):
            for car in cars_ahead:
                other_trajectories[car] = self._estimate_position_rectangles(car, time_step)
        if cars_side and len(cars_side):
            for car in cars_side:
                other_trajectories[car] = self._estimate_position_rectangles(car, time_step)
        final_v = velocity
        final_h = heading
        for i in range(10):
            rvelocity, rheading = self.mode_from_velocity_heading(final_v, final_h)
            max_corn = self._find_max_cornering_acc(mode, rvelocity, rheading, time_step)
            max_long = self._find_max_longitudnal_acc(mode, rvelocity, rheading, time_step)
            print("Resulting V", rvelocity, "Resulting H", rheading)
            if ((rvelocity >= mode[0] and max_long <= self.max_acc) or (rvelocity < mode[0] and max_long <= abs(self.max_brak))
                    and max_corn <= self.max_corn and self._no_collisions_with_cars(car_state, rvelocity, rheading, other_trajectories, time_step)) \
                    or (i == 9):
                print("Current Mode", mode, "Input Mode", (velocity, heading), "Resulting Mode", (rvelocity, rheading))
                return rvelocity, rheading, (rvelocity, rheading)
            else:
                if rvelocity >= mode[0] and max_long < self.max_acc:
                    # Not enough power
                    print("not enough power")
                    best_v = self._find_best_acc(car_state, rvelocity, rheading, other_trajectories, time_step)
                elif rvelocity < mode[0] and max_long <= abs(self.max_brak):
                    # Lock up wheels
                    print("lock up wheels")
                    best_v = self._find_best_braking(car_state, rvelocity, rheading, other_trajectories, time_step)
                else:
                    best_v = rvelocity
                if max_corn > self.max_corn:
                    # understeer not enough aero
                    print("not enough cornering grip")
                    best_h = self._find_best_heading(car_state, rvelocity, rheading, other_trajectories, time_step)
                else:
                    best_h = rheading

                if best_v == mode[0]:
                    final_v = mode[0]
                else:
                    mean_dv = (best_v - mode[0])
                    sd = self.v_prec
                    print("meandv", mean_dv)
                    v_dist = stats.truncnorm((min(mean_dv - 1e-6, 0) - mean_dv) / sd, (max(mean_dv + 1e-6, 0) - mean_dv) / sd, loc=mean_dv, scale=sd)
                    final_v = v_dist.rvs(1)[0] + mode[0]

                if best_h == mode[1]:
                    final_h = mode[1]
                else:
                    mean_dh = self._find_smallest_rotation(best_h, mode[1])
                    sd = self.h_prec
                    print("meandh", mean_dh)
                    h_dist = stats.truncnorm((min(mean_dh - 1e-6, 0) - mean_dh) / sd, (max(mean_dh + 1e-6, 0) - mean_dh) / sd, loc=mean_dh, scale=sd)
                    final_h = h_dist.rvs(1)[0] + mode[1]

                # rvelocity, rheading = final_v, final_h
                # rvelocity, rheading = self.mode_from_velocity_heading(final_v, final_h)


class ControlType(Enum):
    STEER_ACCELERATE = 1
    MODE_ONLY = 2