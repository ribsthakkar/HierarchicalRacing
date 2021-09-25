import math
from enum import Enum
import numpy as np
from scipy import optimize
import scipy.stats as stats
from scipy.optimize import Bounds

from util import round_to_fraction, gravitational_acceleration, find_smallest_rotation, is_cw, TPI, rect_from_center, \
    dist, pos_estimate_functions, is_greater_than, log_leq_barrier_function_value


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

    def _within_braking_limit(self, mode, v, h, dt):
        return not is_greater_than(self._find_max_longitudnal_acc(mode, v, h, dt), abs(self.max_brak), rel_tol=0.01)

    def _within_acc_limit(self, mode, v, h, dt):
        return not is_greater_than(self._find_max_longitudnal_acc(mode, v, h, dt), self.max_acc, rel_tol=0.01)

    def _within_corn_limit(self, mode, v, h, dt):
        return not is_greater_than(self._find_max_cornering_acc(mode, v, h, dt), self.max_corn, rel_tol=0.01)

    def is_transition_feasible(self, vh1, vh2, dt):
        speeding_up = is_greater_than(vh1[0], vh2[0]) or math.isclose(vh1[0], vh2[0])
        return ((speeding_up and self._within_acc_limit(vh1, vh2[0], vh2[1], dt))
                or (not speeding_up and self._within_braking_limit(vh1, vh2[0], vh2[1], dt))) and \
                self._within_corn_limit(vh1, vh2[0], vh2[1], dt)

    def _find_best_collision_avoidance_vh_pair(self, car_state, targetv, targeth, init_v, init_h, other_trajectories, dt):
        mode = car_state.mode
        lb = np.array([0, car_state.mode[1]-TPI])
        ub = np.array([self.max_v, car_state.mode[1]+TPI])
        bounds = Bounds(lb, ub)
        targetx = targetv*math.cos(targeth)
        targety = targetv*math.sin(targeth)
        def opt(x):
            max_c = (max(0, self._find_max_cornering_acc(mode, x[0], x[1], dt) - self.max_corn))**2
            if is_greater_than(x[0], mode[0], rel_tol=0.001) or math.isclose(x[0], mode[0], rel_tol=0.001):
                max_a = (max(0, self._find_max_longitudnal_acc(mode, x[0], x[1], dt) - self.max_acc))**2
            else:
                max_a = (max(0, self._find_max_longitudnal_acc(mode, x[0], x[1], dt) - abs(self.max_brak)))**2
            xx = x[0]*math.cos(x[1])
            xy = x[0]*math.sin(x[1])
            return dist(targetx, targety, xx, xy) + 500*self._area_of_collisions_with_cars(car_state, x[0], x[1], other_trajectories, dt) - 5*x[0] + \
                   0.5*max_c + 0.5*max_a
        print(init_v, init_h)
        result = optimize.minimize(opt, np.array([init_v, init_h]), bounds=bounds)
        return result.x[0], result.x[1]

    def _find_best_heading(self, car_state, targetv, targeth, other_trajectories, dt):
        mode = car_state.mode
        if math.isclose(targeth, mode[1]):
            targeth = mode[1] + 1e-2
            max_bounds = 1e-2
        else:
            max_bounds = find_smallest_rotation(targeth, mode[1])
        if is_cw(mode[1], targeth):
            bounds=(max_bounds, 0)
        else:
            bounds=(0, max_bounds)
        def opt_h(h):
            d = mode[1] + h
            max_c = (max(0, self._find_max_cornering_acc(mode, targetv, d, dt) - self.max_corn))**2
            # area = self._area_of_collisions_with_cars(car_state, targetv, d, other_trajectories, dt)
            return max_c*0.5 + abs(d-targeth)
        result = optimize.minimize_scalar(opt_h, bounds=bounds, method='bounded')
        return result.x + mode[1]

    def _find_best_braking(self, car_state, targetv, targeth, other_trajectories, dt):
        mode = car_state.mode
        if math.isclose(targetv, mode[0]): targetv = mode[0] - 1e-2
        def opt_b(v):
            max_a = (max(0, self._find_max_longitudnal_acc(mode, v, targeth, dt) - self.max_acc)) ** 2
            return max_a * 0.5 + abs(v-targetv)
        result = optimize.minimize_scalar(opt_b, bounds=(targetv, mode[0]), method='bounded')
        return abs(result.x)

    def _find_best_acc(self, car_state, targetv, targeth, other_trajectories, dt):
        mode = car_state.mode
        if math.isclose(targetv, mode[0], rel_tol=0.001): targetv = mode[0] + self.v_prec
        def opt_b(v):
            max_a = (max(0, self._find_max_longitudnal_acc(mode, v, targeth, dt) - self.max_acc)) ** 2
            return max_a * 0.5 + abs(v-targetv)
        result = optimize.minimize_scalar(opt_b, bounds=(mode[0], targetv), method='bounded')
        return abs(result.x)

    def _estimate_position_rectangles(self, state, dt):
        # vi = state.v
        # hi = state.heading
        # hf = math.atan2((state.dy + state.d2y*dt), (state.dx + state.d2x*dt))
        # vf = math.sqrt((state.dy + state.d2y*dt)**2 + (state.dx + state.d2x*dt)**2)
        # x_pos_f, y_pos_f, h_pos_f = self._pos_functions(state.x, state.y, vi, vf, hi, hf, dt)
        # print(state.old_x, state.old_y, state.last_mode[0], state.mode[0], state.last_mode[1], state.mode[1], dt)
        x_pos_f, y_pos_f, h_pos_f = pos_estimate_functions(state.old_x, state.old_y, state.old_v, state.v, state.old_heading, state.heading, dt)
        return [rect_from_center(x, y, state.l, state.w, h) for x, y, h in zip(map(x_pos_f, np.linspace(0, dt)),
                                                                               map(y_pos_f, np.linspace(0, dt)),
                                                                               map(h_pos_f, np.linspace(0, dt)))]

    def _generate_position_rectangles(self, state, targetv, targeth, dt):
        x_pos_f, y_pos_f, h_pos_f = pos_estimate_functions(state.x, state.y, state.v, targetv, state.heading, targeth, dt)
        return [rect_from_center(x, y, state.l, state.w, h) for x, y, h in zip(map(x_pos_f, np.linspace(0, dt)),
                                                                               map(y_pos_f, np.linspace(0, dt)),
                                                                               map(h_pos_f, np.linspace(0, dt)))]

    def _no_collisions_with_cars(self, car_state, targetv, targeth, other_trajectories, dt):
        # to avoid collisions with cars and trajectories we need to make sure bounding boxes don't intersect as traveling
        new_rectangles = self._generate_position_rectangles(car_state, targetv, targeth, dt)
        if not len(other_trajectories): return True
        return not any(ours.intersects(theirs) for c in other_trajectories for ours, theirs in zip(new_rectangles, other_trajectories[c]))

    def _area_of_collisions_with_cars(self, car_state, targetv, targeth, other_trajectories, dt):
        # to avoid collisions with cars and trajectories we need to make sure bounding boxes don't intersect as traveling
        new_rectangles = self._generate_position_rectangles(car_state, targetv, targeth, dt)
        if not len(other_trajectories): return 0
        return sum(ours.intersection(theirs).area for c in other_trajectories for ours, theirs in zip(new_rectangles, other_trajectories[c]))

    def try_switch(self, car_state, velocity, heading, time_step, cars_ahead=None, cars_side=None):
        targetv = velocity
        targeth = heading
        other_trajectories = {}
        mode = car_state.mode
        if cars_ahead and len(cars_ahead):
            for car in cars_ahead:
                other_trajectories[car] = self._estimate_position_rectangles(car, time_step)
        if cars_side and len(cars_side):
            for car in cars_side:
                other_trajectories[car] = self._estimate_position_rectangles(car, time_step)
        best_v = velocity
        best_h = math.fmod(heading, TPI) if heading > 0 else math.fmod(heading+TPI, TPI)
        rvelocity, rheading = self.mode_from_velocity_heading(best_v, best_h)
        max_corn = self._find_max_cornering_acc(mode, rvelocity, rheading, time_step)
        max_long = self._find_max_longitudnal_acc(mode, rvelocity, rheading, time_step)
        speeding_up = (is_greater_than(rvelocity, mode[0], rel_tol=0.001) or math.isclose(rvelocity, mode[0], rel_tol=0.001))
        no_collisions = self._no_collisions_with_cars(car_state, rvelocity, rheading, other_trajectories, time_step)
        within_acceleration = speeding_up and self._within_acc_limit(mode, rvelocity, rheading, time_step)
        within_cornering = self._within_corn_limit(mode, rvelocity, rheading, time_step)
        within_braking = not speeding_up and self._within_braking_limit(mode, rvelocity, rheading, time_step)
        if (within_acceleration or within_braking) and within_cornering and no_collisions:
            print("Current Mode", mode, "Input Mode", (velocity, heading), "Resulting Mode", (rvelocity, rheading))
            return rvelocity, rheading, (rvelocity, rheading)
        else:
            print("Within acceleration limit:" if speeding_up else "Within braking limit: ", within_acceleration if speeding_up else within_braking, max_long, self.max_acc if speeding_up else self.max_brak)
            print("Within cornering limit: ", within_cornering, max_corn, self.max_corn)
            print("No collsions: ", no_collisions)
            if speeding_up and (not within_acceleration):
                # Not enough power
                new_rvelocity = self._find_best_acc(car_state, rvelocity, rheading, other_trajectories, time_step)
            elif not speeding_up and (not within_braking):
                # Lock up wheels
                new_rvelocity = self._find_best_braking(car_state, rvelocity, rheading, other_trajectories, time_step)
            else:
                new_rvelocity = rvelocity
            if not within_cornering:
                # understeer not enough aero
                new_rheading = self._find_best_heading(car_state, rvelocity, rheading, other_trajectories, time_step)
            else:
                new_rheading = rheading
            best_v, best_h = self._find_best_collision_avoidance_vh_pair(car_state, targetv, targeth, new_rvelocity, new_rheading, other_trajectories, time_step)
            mean_dv = (best_v - mode[0])
            sd = self.v_prec
            print("meandv", mean_dv)
            v_dist = stats.truncnorm((min(mean_dv - 1e-6, 0) - mean_dv) / sd,
                                     (max(mean_dv + 1e-6, 0) - mean_dv) / sd, loc=mean_dv, scale=sd)
            final_v = v_dist.rvs(1)[0] + mode[0]

            mean_dh = find_smallest_rotation(best_h, mode[1])
            sd = self.h_prec
            print("meandh", mean_dh)
            h_dist = stats.truncnorm((min(mean_dh - 1e-6, 0) - mean_dh) / sd,
                                     (max(mean_dh + 1e-6, 0) - mean_dh) / sd, loc=mean_dh, scale=sd)
            final_h = h_dist.rvs(1)[0] + mode[1]
            final_h = math.fmod(final_h, TPI) if final_h > 0 else math.fmod(final_h + TPI, TPI)
            rvelocity, rheading = self.mode_from_velocity_heading(final_v, final_h)
            print("Current Mode", mode, "Input Mode", (velocity, heading), "Resulting Mode", (rvelocity, rheading))
            return rvelocity, rheading, (rvelocity, rheading)


class ControlType(Enum):
    STEER_ACCELERATE = 1
    MODE_ONLY = 2