import itertools
import math
from enum import Enum
import numpy as np
from scipy import optimize
import scipy.stats as stats

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
        # mode_pairs = []
        # for v in np.arange(0, max_velocity+vel_precision, vel_precision):
        #     for h in np.arange(0, TPI, heading_precision):
        #         mode_pairs.append((v, h))
        # self.mode_table = nx.Graph()
        # self.mode_table.add_nodes_from(mode_pairs)
        # self.mode_table.add_edges_from(itertools.combinations(mode_pairs, 2))
        # for node in self.mode_table.nodes():
        #     for neighbor in self.mode_table.neighbors(node):
        #         v_diff = neighbor[0] - node[0]
        #         h_diff = min(abs(neighbor[1] + TPI - node[1]), abs(neighbor[1] - node[1]), abs(node[1] + TPI - neighbor[1]), abs(node[1] - neighbor[1]))


    # def generate_world(self, x_size, y_size, x_init, y_init, x_goal, y_goal, weather_sequences, weather_distributions,
    #                    time_horizon, weather_horizon, t_init=1):
    #     builder = stormpy.SparseMatrixBuilder(rows=0, columns=0, entries=0, force_dimensions=False,
    #                                           has_custom_row_grouping=True, row_groups=0)
    #     # base_choice_labels = {"N", "NE", "E", "SE", "S", "SW", "W", "NW", "Stay"}
    #     # state_labels = {(x, y, t): f"({x}, {y}, {t})" for x in range(1, x_size + 1) for y in range(1, y_size + 1) for t
    #     #                 in range(1, time_horizon + 1)}
    #     # choice_labels = {(x, y, t, action): f"({action}, {x}, {y}, {t})" for x in range(1, x_size + 1) for y in
    #     #                  range(1, y_size + 1) for t in range(1, time_horizon + 1) for action in base_choice_labels}
    #     # for x in range(1, x_size+1):
    #     #     for y in range(1, y_size+1):
    #     #         choice_labels[(x, y, time_horizon, "To Sink")] = f"(To Sink, {x}, {y}, {time_horizon})"
    #
    #     total_states = x_size * y_size * time_horizon
    #     state_labeling = stormpy.storage.StateLabeling(total_states + 1)
    #     # for label in state_labels.values():
    #     #     state_labeling.add_label(label)
    #     state_labeling.add_label("sink")
    #     state_labeling.add_label("truegoal")
    #     state_labeling.add_label("init")
    #     sink = 0
    #     count = 0
    #     builder.new_row_group(sink)
    #     builder.add_next_value(count, 1, 1.0)
    #     count += 1
    #     final_choice_counts = {}
    #     state_actions = {}
    #     state_action_rewards = [0]
    #     state_reward = [0.0] * (total_states + 1)
    #     for t in range(1, time_horizon + 1):
    #         for x in range(1, x_size + 1):
    #             for y in range(1, y_size + 1):
    #                 if t < time_horizon:
    #                     tail_state_num = convert_coord_to_state_number(x, y, t, x_size, y_size)
    #                     builder.new_row_group(count)
    #                     neighbors = get_neighbors_for_coord(x, y, x_size, y_size)
    #                     # state_labeling.add_label_to_state(state_labels[(x, y, t)], tail_state_num)
    #                     # print(x, y, t, count)
    #                     state_reward[tail_state_num] = 0
    #                     state_actions[tail_state_num] = []
    #                     if x == x_goal and y == y_goal:
    #                         state_actions[tail_state_num].append("Stay")
    #                         builder.add_next_value(count, convert_coord_to_state_number(x, y, t + 1, x_size, y_size), 1)
    #                         final_choice_counts[(x, y, t, "Stay")] = count
    #                         count += 1
    #                         state_action_rewards.append(0)
    #                     else:
    #                         for n, direction in neighbors.items():
    #                             state_actions[tail_state_num].append(direction)
    #                             if direction == "Stay":
    #                                 builder.add_next_value(count,
    #                                                        convert_coord_to_state_number(n[0], n[1], t + 1, x_size,
    #                                                                                      y_size), 1)
    #                                 final_choice_counts[(x, y, t, direction)] = count
    #                                 state_action_rewards.append(0)
    #                                 count += 1
    #                             else:
    #                                 weather_sequence = weather_sequences[frozenset({n, (x, y)})][
    #                                                    t - 1:t + weather_horizon - 1]
    #                                 on_course_prob = weather_distributions[weather_sequence]
    #                                 # print(x, y, t, direction, on_course_prob, weather_sequence)
    #                                 builder.add_next_value(count,
    #                                                        convert_coord_to_state_number(n[0], n[1], t + 1, x_size,
    #                                                                                      y_size),
    #                                                        on_course_prob)
    #                                 final_choice_counts[(x, y, t, direction)] = count
    #                                 off_course_dirs = get_weather_affected_directions(direction, x, y, x_size, y_size)
    #                                 off_course_prob = (1 - on_course_prob) / len(off_course_dirs)
    #                                 state_action_rewards.append(off_course_prob * 100)
    #                                 for b_n in off_course_dirs:
    #                                     builder.add_next_value(count,
    #                                                            convert_coord_to_state_number(b_n[0], b_n[1], t + 1,
    #                                                                                          x_size, y_size),
    #                                                            off_course_prob)
    #                                 count += 1
    #                 else:
    #                     tail_state_num = convert_coord_to_state_number(x, y, t, x_size, y_size)
    #                     state_actions[tail_state_num] = ["To Sink"]
    #                     builder.new_row_group(count)
    #                     # state_labeling.add_label_to_state(state_labels[(x, y, t)], tail_state_num)
    #                     builder.add_next_value(count, sink, 1)
    #                     final_choice_counts[(x, y, t, 'To Sink')] = count
    #                     state_action_rewards.append(1000000 if not (x == x_goal and y == y_goal) else 0)
    #                     state_reward[tail_state_num] = 0
    #                     count += 1
    #     state_labeling.add_label_to_state("truegoal",
    #                                       convert_coord_to_state_number(x_goal, y_goal, time_horizon, x_size, y_size))
    #     state_labeling.add_label_to_state("init", convert_coord_to_state_number(x_init, y_init, t_init, x_size, y_size))
    #     state_labeling.add_label_to_state("sink", sink)
    #     transition_matrix = builder.build()
    #     reward_models = {}
    #     # state_reward[convert_coord_to_state_number(x_goal, y_goal, time_horizon,x_size, y_size)] = 100000000000
    #
    #     reward_models['goals'] = stormpy.SparseRewardModel(optional_state_reward_vector=state_reward,
    #                                                        optional_state_action_reward_vector=state_action_rewards)
    #     components = stormpy.SparseModelComponents(transition_matrix=transition_matrix, state_labeling=state_labeling,
    #                                                reward_models=reward_models, rate_transitions=False)
    #     # components.choice_labeling = choice_labeling
    #     mdp = stormpy.storage.SparseMdp(components)
    #     return mdp, state_actions, components, final_choice_counts

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

    def _find_best_heading(self, mode, targetv, targeth, dt):
        if math.isclose(targeth, mode[1]): return mode[1]
        max_bounds = self._find_smallest_rotation(targeth, mode[1])
        if self._is_cw(mode[1], targeth): bounds=(-max_bounds, 0)
        else: bounds=(0, max_bounds)
        def opt_h(h):
            d = mode[1] + h
            if self._find_max_cornering_acc(mode, targetv, d, dt) <= self.max_corn:
                return -h
            return 1000000
        result = optimize.minimize_scalar(opt_h, bounds=bounds, method='bounded')
        return -result.x + mode[1] if result.x < 1000000 else mode[1]

    def _find_best_braking(self, mode, targetv, targeth, dt):
        if math.isclose(targetv, mode[0]): return mode[0]
        def opt_b(v):
            if self._find_max_longitudnal_acc(mode, v, targeth, dt) <= abs(self.max_brak):
                return v
            return 1000000
        result = optimize.minimize_scalar(opt_b, bounds=(targetv, mode[0]), method='bounded')
        return result.x if result.x < 1000000 else mode[0]

    def _find_best_acc(self, mode, targetv, targeth, dt):
        if math.isclose(targetv, mode[0]): return mode[0]
        def opt_b(v):
            if self._find_max_longitudnal_acc(mode, v, targeth, dt) <= self.max_acc:
                return -v
            return 10000000
        result = optimize.minimize_scalar(opt_b, bounds=(mode[0], targetv), method='bounded')
        return -result.x if result.x < 1000000 else mode[0]

    def _is_cw(self, current, target):
        return math.sin(current - target) > 0

    def _find_smallest_rotation(self, target_h, current_h):
        print(target_h, current_h)
        if self._is_cw(current_h, target_h):
            return current_h - target_h if current_h > target_h else current_h + TPI - target_h
        else:
            return target_h - current_h if target_h > current_h else target_h + TPI - current_h


    def try_switch(self, mode, velocity, heading, time_step):
        # if heading > TPI: heading = math.fmod(heading, TPI)
        rvelocity, rheading = self.mode_from_velocity_heading(velocity, heading)
        max_corn = self._find_max_cornering_acc(mode, rvelocity, rheading, time_step)
        max_long = self._find_max_longitudnal_acc(mode, rvelocity, rheading, time_step)
        if (rvelocity >= mode[0] and max_long < self.max_acc) or (rvelocity < mode[0] and max_long <= abs(self.max_brak)) \
                and max_corn <= self.max_corn:
            # ss = -math.atan()
            print("Current Mode", mode, "Input Mode", (velocity, heading), "Resulting Mode", (rvelocity, rheading))
            return rvelocity, rheading, (rvelocity, rheading)
        else:
            if rvelocity >= mode[0] and max_long < self.max_acc:
                # Not enough power
                print("not enough power")
                best_v = self._find_best_acc(mode, rvelocity, rheading,time_step)
            elif rvelocity < mode[0] and max_long <= abs(self.max_brak):
                # Lock up wheels
                print("lock up wheels")
                best_v = self._find_best_braking(mode, rvelocity, rheading,time_step)
            else:
                best_v = rvelocity
            if max_corn > self.max_corn:
                # understeer not enough aero
                print("not enough cornering grip")
                best_h = self._find_best_heading(mode, rvelocity, rheading, time_step)
            else:
                best_h = rheading
            print(best_v, best_h)

            if best_v == mode[0]:
                final_v = mode[0]
            else:
                mean_dv = (best_v - mode[0])/2
                print("meandv", mean_dv)
                v_dist = stats.truncnorm((min(best_v - mode[0], 0) - mean_dv) / 1, (max(best_v - mode[0], 0) - mean_dv) / 1, loc=mean_dv, scale=1)
                final_v = v_dist.rvs(1)[0] + mode[0]

            if best_h == mode[1]:
                final_h = mode[1]
            else:
                dh = self._find_smallest_rotation(best_h, mode[1])
                mean_dh = dh/2
                print("meandh", mean_dh)
                h_dist = stats.truncnorm((0 - mean_dh) / 1, (dh - mean_dh) / 1, loc=mean_dh, scale=1)
                final_h = h_dist.rvs(1)[0] + mode[1]

            rvelocity, rheading = self.mode_from_velocity_heading(final_v, final_h)
            print("Current Mode", mode, "Input Mode", (velocity, heading), "Resulting Mode", (rvelocity, rheading))
            return rvelocity, rheading, (rvelocity, rheading)


class ControlType(Enum):
    STEER_ACCELERATE = 1
    MODE_ONLY = 2