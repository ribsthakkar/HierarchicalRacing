import datetime
import math
from collections import deque
from copy import copy

import numpy as np
import shapely.geometry as geom
from scipy.optimize import NonlinearConstraint, Bounds, basinhopping, LinearConstraint

from bezier_util import bezier_acceleration, bezier_speed, bezier_trajectory, bezier_arc_length
from car_modes import DriveModes, ControlType
from util import circ_slice, dist, gravitational_acceleration, TPI, log_leq_barrier_function_value, is_greater_than, \
    log_barrier_function_value


def bezier_race_optimize(agent, opponent_cars, replan_time, input_update_time):
    agent_b_car = BezierCar(agent)
    opp_b_cars = list(map(lambda c: BezierCar(c), opponent_cars))
    if len(opponent_cars) and agent.control_params['optimizer_params']['level'] > 0:
        print("Opponent Optimization")
        for opp in opp_b_cars:
            opp.race_optimize()

        print("Our Optimization given Opponent Optimization Part 1")
        agent_b_car.race_optimize(opp_b_cars)

        if agent.control_params['optimizer_params']['level'] > 1:
            print("Opponent Optimization given Our Optimization")
            opp_b_cars.append(agent_b_car)
            for opp in opp_b_cars:
                if opp == agent_b_car: continue
                opp.race_optimize(opp_b_cars)

            print("Our Optimization given Opponent Optimization Part 2")
            opp_b_cars.remove(agent_b_car)
            agent_b_car.race_optimize(opp_b_cars)
    else:
        agent_b_car.race_optimize([])
    actions = deque()
    t = 0
    while t <= (replan_time) + input_update_time / 2:
        if agent.get_control_type() == ControlType.STEER_ACCELERATE:
            x_p = bezier_speed(agent_b_car.final_cp[:agent_b_car.num_cp], t, agent_b_car.bezier_order, agent_b_car.time_horizon)
            x_pp = bezier_acceleration(agent_b_car.final_cp[:agent_b_car.num_cp], t, agent_b_car.bezier_order, agent_b_car.time_horizon)
            y_p = bezier_speed(agent_b_car.final_cp[agent_b_car.num_cp:agent_b_car.num_cp * 2], t, agent_b_car.bezier_order, agent_b_car.time_horizon)
            y_pp = bezier_acceleration(agent_b_car.final_cp[agent_b_car.num_cp:agent_b_car.num_cp * 2], t, agent_b_car.bezier_order, agent_b_car.time_horizon)
            sp = (x_p ** 2 + y_p ** 2)
            if sp > 0:
                acceleration = (x_p * x_pp + y_p * y_pp) / math.sqrt(sp)
            else:
                acceleration = 0
            numerator = (x_p*y_pp - y_p*x_pp)
            if sp != 0:
                st_an = math.atan((numerator/(sp **1.5)) * agent_b_car.car_length)
            else:
                st_an = 0
            actions.append((acceleration, st_an, DriveModes.RACE))
        elif agent.get_control_type() == ControlType.MODE_ONLY:
            x_p = bezier_speed(agent_b_car.final_cp[:agent_b_car.num_cp], t, agent_b_car.bezier_order,
                               agent_b_car.time_horizon)
            y_p = bezier_speed(agent_b_car.final_cp[agent_b_car.num_cp:agent_b_car.num_cp * 2], t,
                               agent_b_car.bezier_order, agent_b_car.time_horizon)
            sp = math.sqrt(x_p ** 2 + y_p ** 2)
            heading = math.atan2(y_p, x_p)
            if heading < 0: heading = heading + TPI
            actions.append((sp, heading))
        else:
            print("Unknown control type")
            exit(1)
        t += input_update_time

    return actions

class BezierCar:
    def __init__(self, agent):
        parameters = agent.control_params['optimizer_params']
        self.min_point_horizon = parameters['min_point_horizon']
        self.max_point_horizon = parameters['max_point_horizon']
        self.bezier_order = parameters['bezier_order']
        self.num_cp = self.bezier_order + 1
        self.plan_time_delta = parameters['plan_time_precision']
        self.time_horizon = parameters['plan_time_horizon']
        self.ipx = agent.state.tpx
        self.x = agent.state.x
        self.y = agent.state.y
        self.dx = agent.state.dx
        self.dy = agent.state.dy
        self.d2x = agent.state.d2x
        self.d2y = agent.state.d2y
        self.max_gs = agent.max_gs
        self.max_steering_angle = agent.max_steering_angle
        self.acceleration_bound = agent.acc_profile
        self.max_braking = agent.max_braking
        self.max_vel = agent.max_vel
        self.car_width = agent.width
        self.car_length = agent.length
        self.horizon_increment = 5
        self.final_cp = ([self.x] * self.num_cp) + ([self.y] * self.num_cp) + ([25])
        self.track = agent.track

    def compute_trajectory_intersection(self, x1, y1, x2, y2, x3, y3, x4, y4):
        dx1 = x2 - x1
        dy1 = y2 - y1
        dx2 = x4 - x3
        dy2 = y4 - y3
        # print(dx1, dy1, dx2, dy2)
        if (math.isclose(dx1, dx2, abs_tol=1e-5) and math.isclose(dy1, dy2, abs_tol=1e-5)) or \
            (math.isclose(dx1, -dx2, abs_tol=1e-5) and math.isclose(dy1, -dy2, abs_tol=1e-5)) or \
                (math.isclose(dx1, 0, abs_tol=1e-5) and math.isclose(dy1, 0, abs_tol=1e-5)) or \
                (math.isclose(dx2, 0, abs_tol=1e-5) and math.isclose(dy2, 0, abs_tol=1e-5)) or \
                (math.isclose((x1-x2)*(y3-y4) - (y1-y2) * (x3-x4), 0, abs_tol=1e-5)):
            # print("singular")
            intersect_x = (x3 + x2) / 2
            intersect_y = (y3 + y2) / 2
            # print(intersect_x, intersect_y)
            dir_x = dx1
            dir_y = dy1
        else:
            # print((x1-x2)*(y3-y4) - (y1-y2) * (x3-x4))
            # print("non-singular")
            intersect_x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2)*(x3*y4 - y3*x4))/((x1-x2)*(y3-y4) - (y1-y2) * (x3-x4))
            intersect_y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2)*(x3*y4 - y3*x4))/((x1-x2)*(y3-y4) - (y1-y2) * (x3-x4))
            # intersect_x = (x3 + x2) / 2
            # intersect_y = (y3 + y2) / 2
            len_1 = math.sqrt(dx1**2 + dy1**2)
            len_2 = math.sqrt(dx2**2 + dy2**2)
            dir_x = dx1 - dx2 * (len_1/len_2)
            dir_y = dy1 - dy2 * (len_1/len_2)
        # print("intersect_x", intersect_x, "intersect_y", intersect_y, "dir_x", dir_x, "dir_y", dir_y)
        if not ((math.isclose(dir_x, 0, abs_tol=1e-5)) or (math.isclose(dir_y, 0, abs_tol=1e-5))):
            if dir_x < 0:
                x_target = 0
            else:
                x_target = 850

            if dir_y < 0:
                y_target = 0
            else:
                y_target = 650
            tx = (x_target - intersect_x)/dir_x
            ty = (y_target-intersect_y)/dir_y
            if tx < ty:
                return intersect_x + tx*dir_x/2, intersect_y + tx*dir_y/2
            else:
                return intersect_x + ty*dir_x/2, intersect_y + ty*dir_y/2
        elif (math.isclose(dir_x, 0, abs_tol=1e-5)) and (math.isclose(dir_y, 0, abs_tol=1e-5)):
            return intersect_x, intersect_y
        elif math.isclose(dir_y, 0, abs_tol=1e-5):
            if dir_x < 0:
                x_target = 0
            else:
                x_target = 850
            tx = (x_target - intersect_x)/dir_x
            return intersect_x + tx*dir_x/2, intersect_y + tx*dir_y/2
        else:
            if dir_y < 0:
                y_target = 0
            else:
                y_target = 850
            ty = (y_target-intersect_y)/dir_y
            return intersect_x + ty*dir_x/2, intersect_y + ty*dir_y/2
        # return intersect_x + dir_x, intersect_y + dir_y

    def take_step(self, x):
        self.min_point_horizon += self.horizon_increment
        target_x = self.track.center_x[(self.ipx + self.min_point_horizon) % len(self.track.center_x)]
        target_y = self.track.center_y[(self.ipx + self.min_point_horizon) % len(self.track.center_y)]
        target_x1 = self.track.center_x[(self.ipx + self.min_point_horizon - 1) % len(self.track.center_x)]
        target_y1 = self.track.center_y[(self.ipx + self.min_point_horizon - 1) % len(self.track.center_y)]
        x[self.num_cp - 2] = target_x1
        x[self.num_cp * 2 - 2] = target_y1
        x[self.num_cp - 1] = target_x
        x[self.num_cp * 2 - 1] = target_y
        self.lb[self.num_cp - 1] = target_x
        self.lb[self.num_cp * 2 - 1] = target_y
        self.ub[self.num_cp - 1] = target_x + 1e-6
        self.ub[self.num_cp * 2 - 1] = target_y + 1e-6
        x[self.num_cp * 2] = self.min_point_horizon
        self.bounds = Bounds(self.lb, self.ub)
        return x

    def prep_control_points(self):
        self.min_point_horizon = 25 #int(min(base_min_point_horizon, 0.5 * ((self.time_horizon-1) ** 2) * math.sqrt(self.d2x**2 + self.d2y**2)))
        print("min_pt_hz", self.min_point_horizon, "max_pt_hz", self.max_point_horizon)
        init_dx_dt = self.dx
        init_dy_dt = self.dy
        init_d2x_dt = self.d2x
        init_d2y_dt = self.d2y
        init_position_index = self.ipx
        init_x = self.x
        init_y = self.y
        target_x = self.track.center_x[(self.ipx + self.min_point_horizon)%len(self.track.center_x)]
        target_y = self.track.center_y[(self.ipx + self.min_point_horizon)%len(self.track.center_y)]
        target_x1 = self.track.center_x[(self.ipx + self.min_point_horizon - 1)%len(self.track.center_x)]
        target_y1 = self.track.center_y[(self.ipx + self.min_point_horizon - 1)%len(self.track.center_y)]
        c1x = (init_x + self.time_horizon * init_dx_dt / self.bezier_order)
        c1y = (init_y + self.time_horizon * init_dy_dt / self.bezier_order)
        c2x = self.time_horizon ** 2 * init_d2x_dt / (self.bezier_order * (self.bezier_order - 1)) + 2 * (c1x) - init_x
        c2y = self.time_horizon ** 2 * init_d2y_dt / (self.bezier_order * (self.bezier_order - 1)) + 2 * (c1y) - init_y
        c_x = [init_x, c1x, c2x]
        c_y = [init_y, c1y, c2y]

        inter_x, inter_y = self.compute_trajectory_intersection(init_x, init_y, c1x, c1y, target_x1, target_y1, target_x, target_y)
        for i in range(3, self.num_cp-2):
            c_x.append(inter_x)
            c_y.append(inter_y)

        c_x.append(target_x1)
        c_y.append(target_y1)
        c_x.append(target_x)
        c_y.append(target_y)
        print(c_x)
        print(c_y)
        # exit(1)


        lb = [init_x, c1x, c2x]
        for i in range(3, self.num_cp-1): lb.append(0)
        lb.append(target_x)
        lb += [init_y, c1y, c2y]
        for i in range(3, self.num_cp-1): lb.append(0)
        lb.append(target_y)
        lb.append(self.min_point_horizon)
        ub = [init_x+1e-6, c1x+1e-6, c2x+1e-6]
        for i in range(3, self.num_cp-1): ub.append(850)
        ub.append(target_x)
        ub += [init_y+1e-6, c1y+1e-6, c2y+1e-6]
        for i in range(3, self.num_cp-1): ub.append(650)
        ub.append(target_y)
        ub.append(self.max_point_horizon)

        c = c_x + c_y
        c.append(self.min_point_horizon)
        initial = np.array(c)
        self.initial = initial
        self.ub = ub
        self.lb = lb
        self.track_horizon = geom.LineString(circ_slice(self.track.center_coords, init_position_index, 200))

    def race_optimize(self, opponent_cars=None):
        if opponent_cars:
            opponent_cars = list(filter(lambda c: c != self, opponent_cars))
        else:
            opponent_cars = []
        self.prep_control_points()
        c_x = self.initial[:self.num_cp]
        c_y = self.initial[self.num_cp:self.num_cp*2]
        def opt(c):
            total = 0
            t = self.plan_time_delta
            while t <= self.time_horizon + self.plan_time_delta / 2:
                if t > self.time_horizon - self.plan_time_delta / 2: t = self.time_horizon
                x_p = bezier_speed(c[:self.num_cp], t, self.bezier_order, self.time_horizon)
                x_pp = bezier_acceleration(c[:self.num_cp], t, self.bezier_order, self.time_horizon)
                y_p = bezier_speed(c[self.num_cp:self.num_cp*2], t, self.bezier_order, self.time_horizon)
                y_pp = bezier_acceleration(c[self.num_cp:self.num_cp*2], t, self.bezier_order, self.time_horizon)
                sp = math.sqrt(x_p ** 2 + y_p ** 2)
                sp1 = math.sqrt((x_p + x_pp*self.plan_time_delta)**2 + (y_p + y_pp*self.plan_time_delta)**2)
                ac = math.sqrt(x_pp ** 2 + y_pp ** 2)

                long_acc = 0.5 * (2 * x_pp * (x_p) + 2 * y_pp * (y_p)) / (math.sqrt((x_p) ** 2 + (y_p) ** 2))
                if math.isclose(ac, abs(long_acc)) or math.isnan(long_acc) or math.isinf(long_acc):
                    long_acc = ac
                lat_acc = math.sqrt(ac**2 - long_acc**2)

                def acc_penalty(c):
                    if is_greater_than(sp1, sp, rel_tol=0.001) or math.isclose(sp1, sp, rel_tol=0.001):
                        return min(0, abs(self.max_braking) - long_acc) + min(0, self.max_gs*gravitational_acceleration - lat_acc)
                    else:
                        return min(0, self.acceleration_bound(sp) - long_acc) + min(0, self.max_gs*gravitational_acceleration - lat_acc)

                if opponent_cars:
                    for o in opponent_cars:
                        con4 = lambda c: dist(bezier_trajectory(c[:self.num_cp], t, self.bezier_order, self.time_horizon),
                                              bezier_trajectory(c[self.num_cp:self.num_cp*2], t, self.bezier_order, self.time_horizon),
                                              bezier_trajectory(o.final_cp[:self.num_cp], t, self.bezier_order, self.time_horizon),
                                              bezier_trajectory(o.final_cp[self.num_cp:self.num_cp*2], t, self.bezier_order, self.time_horizon))
                        total -= (500) * min(0, con4(c) - (math.sqrt(self.car_width**2 + self.car_length**2)))
                total -= 50 * acc_penalty(c)
                # total -= 10 * vel_penalty(c)
                # total -= 10 * steer_penalty(c)
                total -= 20/(self.time_horizon/self.plan_time_delta) * sp
                total += 45 * self.track.distance_to_center_custom_range(bezier_trajectory(c[:self.num_cp], t, self.bezier_order, self.time_horizon),
                                                                         bezier_trajectory(c[self.num_cp:self.num_cp*2], t, self.bezier_order, self.time_horizon),
                                                                         self.ipx, self.ipx + self.max_point_horizon)
                t += self.plan_time_delta
            if opponent_cars:
                for opp in opponent_cars:
                    total -= (6 / len(opponent_cars)) * (
                                self.track.find_pos_index(self.ipx, c[self.num_cp-1], c[self.num_cp*2-1]) -
                                self.track.find_pos_index(opp.ipx, opp.final_cp[self.num_cp-1], opp.final_cp[self.num_cp*2-1]))
            total -= 0.2 * (c[-1] - self.ipx)
            return total

        constraints = []

        # Speed Constraints
        t = 0
        while t <= self.time_horizon + self.plan_time_delta / 2:
            if t > self.time_horizon - self.plan_time_delta / 2: t = self.time_horizon
            con = lambda c: bezier_speed(c[:self.num_cp], t, self.bezier_order, self.time_horizon) ** 2 + bezier_speed(c[self.num_cp:self.num_cp*2], t, self.bezier_order, self.time_horizon) ** 2
            nlc = NonlinearConstraint(con, 0, self.max_vel ** 2)
            constraints.append(nlc)

            t += self.plan_time_delta

        # Steering Angle Constraints
        t = 0
        while t <= self.time_horizon + self.plan_time_delta / 2:
            if t > self.time_horizon - self.plan_time_delta / 2: t = self.time_horizon

            def con(c):
                x_p = bezier_speed(c[:self.num_cp], t, self.bezier_order, self.time_horizon)
                x_pp = bezier_acceleration(c[:self.num_cp], t, self.bezier_order, self.time_horizon)
                y_p = bezier_speed(c[self.num_cp:self.num_cp*2], t, self.bezier_order, self.time_horizon)
                y_pp = bezier_acceleration(c[self.num_cp:self.num_cp*2], t, self.bezier_order, self.time_horizon)
                sp = (x_p ** 2 + y_p ** 2)
                return math.atan(((x_p * y_pp - y_p * x_pp) / (sp ** 1.5)) * self.car_length) * 180 / math.pi

            nlc = NonlinearConstraint(con, -self.max_steering_angle, self.max_steering_angle)
            constraints.append(nlc)
            t += self.plan_time_delta

        con = lambda c: self.track.find_pos_index(self.ipx, c[self.num_cp-1],c[self.num_cp*2 -1]) - self.ipx
        nlc = NonlinearConstraint(con, self.min_point_horizon, self.max_point_horizon)
        constraints.append(nlc)
        print("Initial x speed", bezier_speed(self.initial[:self.num_cp], 0, self.bezier_order, self.time_horizon), "Initial y speed",
              bezier_speed(self.initial[self.num_cp:self.num_cp*2], 0, self.bezier_order, self.time_horizon),
              "Initial x acc", bezier_acceleration(self.initial[:self.num_cp], 0, self.bezier_order, self.time_horizon), "Initial y acc",
              bezier_acceleration(self.initial[self.num_cp:self.num_cp*2], 0, self.bezier_order, self.time_horizon))
        print("init control x=", c_x)
        print("init control y=", c_y)
        print("init lb=", self.lb)
        print("init ub=", self.ub)
        self.bounds = Bounds(self.lb, self.ub)
        print(datetime.datetime.now())
        result = basinhopping(opt, self.initial, niter=10, minimizer_kwargs={'bounds': self.bounds, 'constraints': constraints, 'method': 'SLSQP',
                                                                       'options': {'maxiter': 10}}, take_step=self.take_step)
        print(datetime.datetime.now())
        if math.nan not in result.x and math.inf not in result.x:
            self.final_cp = result.x
        else:
            self.final_cp = self.initial
        output = [round(x, 3) for x in result.x]
        print("c_x", output[:len(c_x)])
        print("c_y", output[len(c_x):len(c_x) + len(c_y)])
        print("N", output[-1])
        print("planned trajectory distance:", bezier_arc_length(self.final_cp[:len(c_x)], self.final_cp[len(c_x):len(c_x) + len(c_y)], time_horizon=self.time_horizon))
