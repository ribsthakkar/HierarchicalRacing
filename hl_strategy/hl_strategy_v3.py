import itertools
import math
from datetime import datetime
from enum import Enum
from functools import lru_cache
from typing import Dict, List, TextIO

import numpy as np
from scipy.optimize import NonlinearConstraint, Bounds, minimize

from util import _write_with_newline_and_sc, neg_to_str, is_greater_than

MAX_TIRE_AGE = 100
RUN_STORMPY = False

@lru_cache(False)
def calc_min_time(u, v, s, a_min, a_max, v_max):
    constraints = []

    def opt(x):
        return x[2]

    def time_variables_constraint(x):
        return x[1] - x[0]
    nlc = NonlinearConstraint(time_variables_constraint, 0, np.inf)
    constraints.append(nlc)

    def time_variables_constraint2(x):
        return x[2] - x[1]
    nlc = NonlinearConstraint(time_variables_constraint2, 0, np.inf)
    constraints.append(nlc)

    def distance_constraint(x):
        iv = u + a_max*x[0]
        dist = (u+iv)*x[0]/2
        dist += (x[1]-x[0])*iv
        t2 = (v-iv)/-abs(a_min)
        dist += (iv + v)*t2
        t3 = x[2]-x[1]
        dist += (v)*t3
        return dist
    nlc = NonlinearConstraint(distance_constraint, s, s)
    constraints.append(nlc)

    def max_vel_constraint(x):
        iv = u+ a_max*x[0]
        return iv
    nlc = NonlinearConstraint(max_vel_constraint, 0, v_max)
    constraints.append(nlc)

    lb = [0, 0, 0]
    ub = [100, 100, 100]
    bounds = Bounds(lb, ub)
    x0 = [0,0.1,0.1]
    result = minimize(opt, x0, constraints=constraints, bounds=bounds)
    if (is_greater_than(max_vel_constraint(result.x), v_max, abs_tol=1e-4) or not math.isclose(distance_constraint(result.x), s, abs_tol=1e-4) or result.x[0] <= -1e-5 or result.x[1] <= -1e-5):
        print(max_vel_constraint(result.x), distance_constraint(result.x), s, u, v, result.x)
        print()
        return None
    return result.fun


class CarDef:
    def __init__(self, max_velocity, velocity_step, min_gs, max_gs, max_braking, max_acceleration, tire_wear_factor,
                 init_tire, init_time, init_line, init_velocity, init_position):
        self.max_v = max_velocity
        self.velocity_step = velocity_step
        self.max_braking = -abs(max_braking)
        self.max_acceleration = max_acceleration
        self.min_gs = min_gs
        self.max_gs = max_gs
        self.init_tire = init_tire
        self.init_time = init_time
        self.init_line = init_line
        self.init_velocity = init_velocity
        self.init_position = init_position
        self.tire_wear_factor = tire_wear_factor


class TrackComponent:
    def __init__(self, length):
        self.length = length

    def is_v_feasible(self, velocity, line, tire_wear, min_cornering_gs, max_cornering_gs):
        raise NotImplementedError("Must be implemented by child classes")

    def tire_wear(self, velocity, line, tire_wear_factor):
        raise NotImplementedError("Must be implemented by child classes")


class TrackStraight(TrackComponent):
    def __init__(self, length):
        super().__init__(length)

    def is_v_feasible(self, velocity, line, tire_wear, min_cornering_gs, max_cornering_gs):
        return True

    def tire_wear(self, velocity, line, tire_wear_factor):
        if velocity == 0: return 0
        return 1


class TrackCorner:
    class Entry(TrackComponent):
        def __init__(self, length, turn_radius, width, num_lines, left_turn=True):
            super().__init__(length)
            order = range(num_lines) if left_turn else reversed(range(num_lines))
            self.tr = list(map(lambda l: turn_radius + l*width/num_lines, order))

        def is_v_feasible(self, velocity, line, tire_wear, min_cornering_gs, max_cornering_gs):
            gs = (velocity**2)/self.tr[line]
            g_diff = (max_cornering_gs-min_cornering_gs)*(tire_wear/MAX_TIRE_AGE)
            return gs <= max_cornering_gs-g_diff

        def tire_wear(self, velocity, line, tire_wear_factor):
            gs = (velocity**2)/self.tr[line]
            return int((math.ceil(gs) * self.length/velocity)/tire_wear_factor)


    class Mid(TrackComponent):
        def __init__(self, length, turn_radius, width, num_lines, left_turn=True):
            super().__init__(length)
            order = range(num_lines) if left_turn else reversed(range(num_lines))
            self.tr = list(map(lambda l: turn_radius + l*width/num_lines, order))

        def is_v_feasible(self, velocity, line, tire_wear, min_cornering_gs, max_cornering_gs):
            gs = (velocity**2)/self.tr[line]
            g_diff = (max_cornering_gs-min_cornering_gs)*(tire_wear/MAX_TIRE_AGE)
            return gs <= max_cornering_gs-g_diff

        def tire_wear(self, velocity, line, tire_wear_factor):
            gs = (velocity**2)/self.tr[line]
            return math.ceil((math.ceil(gs) * self.length/velocity)/tire_wear_factor)


    def __init__(self, num_lines, width, inside_turn_radius, degrees, exit_length, left_turn=True):
        turn_length = math.radians(degrees) * inside_turn_radius
        self.entry = self.Entry(turn_length/2, inside_turn_radius, width, num_lines, left_turn)
        self.mid = self.Mid(turn_length/2, inside_turn_radius, width, num_lines, left_turn)
        self.exit = TrackStraight(exit_length)


class TrackDef:
    def __init__(self, track_landmarks, width, num_lanes, pit_exit_position, pit_exit_velocity, pit_exit_line, pit_time):
        self.num_lanes = num_lanes
        self.width = width
        self.pit_exit_p = pit_exit_position
        self.pit_exit_l = pit_exit_line
        self.pit_exit_v = pit_exit_velocity
        self.pit_time = pit_time
        self.landmarks = self._process_landmarks(track_landmarks)
        self.track_points= len(self.landmarks)

    def _process_landmarks(self, input_landmarks):
        output = []
        for l in input_landmarks:
            if type(l) == TrackStraight:
                output.append(l)
            elif type(l) == TrackCorner:
                output.append(l.entry)
                output.append(l.mid)
                output.append(l.exit)
        return output



class TimePrecision(Enum):
    Hundredths = 100
    Tenths = 10
    Seconds = 1


def generate_modules(output_file, total_seconds, laps, track_definition, car_definitions, time_precision, crash_tolerance=1, is_game=False):
    with open(output_file, "w+") as output:
        if not is_game:
            _write_with_newline_and_sc("mdp\n", output, False)
        else:
            _write_with_newline_and_sc("csg\n", output, False)

        tps = track_definition.track_points
        tls = track_definition.num_lanes
        pit_out = track_definition.pit_exit_p
        pit_out_v = track_definition.pit_exit_v
        pit_out_l = track_definition.pit_exit_l
        max_time = total_seconds * time_precision.value
        pit_time = track_definition.pit_time * time_precision.value
        for idx, car_definition in enumerate(car_definitions):
            max_v = car_definition.max_v
            init_tire = car_definition.init_tire
            init_time = car_definition.init_time
            init_line = car_definition.init_line
            init_v = car_definition.init_velocity
            init_pos = car_definition.init_position
            velocity_step = car_definition.velocity_step

            # Define Fixed Action Set
            action_set = {}
            for velocity in range(1, max_v+math.ceil(velocity_step//2), car_definition.velocity_step):
                ub = min(max_v+1, velocity+velocity_step)
                for lane in range(tls):
                    action_set[(velocity, ub, lane)] = f"[step{idx}_b{velocity}_a{ub}_l{lane}]"

            # Define Track Section Formulas
            for i in range(tps):
                for action in action_set:
                    avg_v = (action[0] + action[1])/2
                    target_section = track_definition.landmarks[(i+1)%tps]
                    for ta in range(0, MAX_TIRE_AGE+1):
                        if not target_section.is_v_feasible(avg_v, action[2], ta, car_definition.min_gs, car_definition.max_gs): break
                    _write_with_newline_and_sc(f"formula sec{i}_b{action[0]}_a{action[1]}_l{action[2]}_c{idx} = tire_age{idx} < {ta} & track_pos{idx}={i}", output)

            # Define Formulas for allowed actions
            action_allowed_strings = {}
            for action in action_set:
                action_allowed_strings[action] = f"b{action[0]}_a{action[1]}_l{action[2]}_c{idx}"
                _write_with_newline_and_sc(f"formula b{action[0]}_a{action[1]}_l{action[2]}_c{idx} = {' | '.join(map(lambda i:f'sec{i}_b{action[0]}_a{action[1]}_l{action[2]}_c{idx}', range(tps)))}", output)


            # Position/Lap Module
            _write_with_newline_and_sc(f'module progress{idx}\n', output, False)
            _write_with_newline_and_sc(f'track_pos{idx} : [0..{tps-1}] init {init_pos}', output)
            _write_with_newline_and_sc(f'lap{idx} : [0..{laps}] init 0', output)
            for action_string in action_set.values():
                _write_with_newline_and_sc(f"{action_string} track_pos{idx} < {tps-1} & lap{idx} < {laps} ->  1: (track_pos{idx}'=track_pos{idx}+1)", output)
                _write_with_newline_and_sc(f"{action_string} track_pos{idx} = {tps-1} & lap{idx} < {laps} ->  1: (track_pos{idx}'=0) & (lap{idx}' = lap{idx}+1)", output)
            _write_with_newline_and_sc(
                f"[worn_{idx}] track_pos{idx} < {tps - 1} & lap{idx} < {laps} ->  1: (track_pos{idx}'=track_pos{idx}+1)",
                output)
            _write_with_newline_and_sc(
                f"[worn_{idx}] track_pos{idx} = {tps - 1} & lap{idx} < {laps} ->  1: (track_pos{idx}'=0) & (lap{idx}' = lap{idx}+1)",
                output)

            _write_with_newline_and_sc(f"[pit_{idx}] track_pos{idx} = {tps-1} & lap{idx} < {laps} -> 1: (track_pos{idx}'={pit_out}) & (lap{idx}' = lap{idx}+1)", output)
            _write_with_newline_and_sc("endmodule", output, False)

            # Tire Age Module
            _write_with_newline_and_sc(f'module tire_wear{idx}\n', output, False)
            _write_with_newline_and_sc(f'tire_age{idx} : [0..{MAX_TIRE_AGE+100}] init {init_tire}', output)
            for i, section in enumerate(track_definition.landmarks):
                for action, action_string in action_set.items():
                    action_str = f"{action_string}"
                    max_dta = 0
                    updates = []
                    for v in range(action[0], action[1]):
                        dta = section.tire_wear(v, action[2], car_definition.tire_wear_factor)
                        max_dta = max(max_dta, dta)
                        updates.append(f"(tire_age{idx}'=tire_age{idx}+{dta})")
                    prob_str = f"1/{len(updates)}"
                    for index in range(len(updates)):
                        updates[index] = f"{prob_str}:{updates[index]}"
                    guard_str = f"track_pos{idx}={i} & tire_age{idx} < {MAX_TIRE_AGE+100-max_dta}"
                    if len(updates):
                        _write_with_newline_and_sc(f"{action_str} {guard_str} -> {' + '.join(updates)}", output)
            _write_with_newline_and_sc(f"[worn_{idx}] true ->  1: (tire_age{idx}'=tire_age{idx})", output)
            _write_with_newline_and_sc(f"[pit_{idx}] track_pos{idx}={tps-1} -> 1: (tire_age{idx}'=0)", output)
            _write_with_newline_and_sc("endmodule", output, False)

            # Car Module
            _write_with_newline_and_sc(f'module racecar{idx}\n', output, False)
            _write_with_newline_and_sc(f't{idx} : [0..{max_time}] init {init_time}', output)
            _write_with_newline_and_sc(f'track_lane{idx} : [0..{max(1, tls-1)}] init {init_line}', output)
            _write_with_newline_and_sc(f'velocity{idx} : [1..{max_v}] init {init_v}', output)
            for section_idx, section in enumerate(track_definition.landmarks):
                for cur_v in range(1, car_definition.max_v+math.ceil(velocity_step//2), velocity_step):
                    for cur_lane in range(tls):
                        for action, action_string in action_set.items():
                            action_str = f"{action_string}"
                            avg_init_v = (cur_v + cur_v+velocity_step)/2
                            max_dt = 0
                            updates = []
                            dist = math.sqrt(
                                (section.length) ** 2 + (abs(cur_lane - action[2]) * (track_definition.width) / (tls + 1)) ** 2)
                            for v in range(action[0], action[1]):
                                min_time = calc_min_time(avg_init_v, v, dist,
                                                         car_definition.max_braking, car_definition.max_acceleration, car_definition.max_v)
                                if min_time is None: continue
                                else: dt = math.ceil(min_time*time_precision.value)
                                max_dt = max(max_dt, dt)
                                updates.append(f"(velocity{idx}'={v})&(track_lane{idx}'={action[2]})&(t{idx}'=t{idx}+{dt})")
                            prob_str = f"1/{len(updates)}"
                            for i in range(len(updates)):
                                updates[i] = f"{prob_str}:{updates[i]}"
                            guard_str = f"{action_allowed_strings[action]} & track_lane{idx}={cur_lane} & t{idx}<{max_time-max_dt} & velocity{idx}>={cur_v} & velocity{idx} < {cur_v + velocity_step} & track_pos{idx}={section_idx}"
                            if len(updates):
                                _write_with_newline_and_sc(f"{action_str} {guard_str} -> {' + '.join(updates)}", output)
                action = f"[worn_{idx}]"
                guard = f"tire_age{idx}>={MAX_TIRE_AGE} & t{idx} < {int(section.length)} & track_pos{idx}={section_idx}"
                _write_with_newline_and_sc(f"{action} {guard} -> 1:(velocity{idx}'=1) & (track_lane{idx}'=track_lane{idx}) & (t{idx}'=t{idx}+{int(section.length * time_precision.value)})", output)
            action = f"[pit_{idx}]"
            guard = f"track_pos{idx}={tps-1} & t{idx}<{max_time-pit_time}"
            _write_with_newline_and_sc(
                f"{action} {guard} -> 1:(velocity{idx}'={min(pit_out_v, max_v)}) & (track_lane{idx}'={pit_out_l}) & (t{idx}'=t{idx}+{pit_time})", output)
            _write_with_newline_and_sc("endmodule", output, False)

            _write_with_newline_and_sc("", output, False)
            _write_with_newline_and_sc(f'label \"goal{idx}\" = (lap{idx}={laps} & track_pos{idx}=0) | (lap{idx}={laps} & track_pos{idx}={track_def.pit_exit_p})', output)

            _write_with_newline_and_sc(f"rewards \"total_time{idx}\"", output, False)
            _write_with_newline_and_sc(f"lap{idx}={laps-1} & track_pos{idx}={tps-1}: t{idx}", output)
            _write_with_newline_and_sc("endrewards", output, False)
            if is_game:
                _write_with_newline_and_sc(f"player p{idx}", output, False)
                _write_with_newline_and_sc(f"racecar{idx}, {', '.join(action_set.values())}", output, False)
                _write_with_newline_and_sc("endplayer", output, False)

        if len(car_definitions) > 1:
            crash_strings = []
            for pair in itertools.combinations(list(range(len(car_definitions))), 2):
                crash_strings.append(f"((track_pos{pair[0]} = track_pos{pair[1]}) & (t{pair[0]}-t{pair[1]}<={crash_tolerance*time_precision.value} & t{pair[0]}-t{pair[1]} >=-{crash_tolerance*time_precision.value}))")
            _write_with_newline_and_sc(f"label crash = {' | '.join(crash_strings)}", output)


if __name__ == "__main__":
    LANES = 2
    WIDTH = 5
    LENGTH = 2
    pit_exit_p = 1
    pit_exit_v = 3
    pit_exit_line = 0
    pit_time = 5
    # components = [(3, np.inf), # 6 m straight,
    #               (2, 2.5, True), # 4 m corner
    #               (1, np.inf), # 2 m straight
    #               (2, 2.5, True),
    #               (3, np.inf),
    #               (2, 2.5, True),
    #               (1, np.inf),
    #               (2, 2.5, True)]
    # track_def = TrackDef(components, component_length=LENGTH, width=WIDTH, num_lanes=LANES, pit_exit_position=pit_exit_p, pit_exit_velocity=pit_exit_v, pit_exit_line=pit_exit_line, pit_time=pit_time)
    components = [TrackStraight(5), TrackCorner(num_lines=LANES, width=WIDTH, inside_turn_radius=2.5, degrees=90, exit_length=2.3),
                  TrackCorner(num_lines=LANES, width=WIDTH, inside_turn_radius=2.5, degrees=90, exit_length=5.0), TrackStraight(5.0),
                  TrackCorner(num_lines=LANES, width=WIDTH, inside_turn_radius=2.5, degrees=90, exit_length=2.3),
                  TrackCorner(num_lines=LANES, width=WIDTH, inside_turn_radius=2.5, degrees=90, exit_length=5.0)]
    track_def = TrackDef(components, width=WIDTH, num_lanes=LANES, pit_exit_position=pit_exit_p, pit_exit_velocity=pit_exit_v, pit_exit_line=pit_exit_line, pit_time=pit_time)
    car_def1 = CarDef(max_velocity=5, velocity_step=1, min_gs=0.1*9.8, max_gs=0.3*9.8, max_braking=4, max_acceleration=2,
                     tire_wear_factor=.7,
                     init_time=0, init_tire=0, init_line=0, init_velocity=1, init_position=0)
    car_def2 = CarDef(max_velocity=10, velocity_step=2, min_gs=0.1*9.8, max_gs=0.3*9.8, max_braking=2, max_acceleration=2,
                     tire_wear_factor=1,
                     init_time=0, init_tire=0, init_line=1, init_velocity=3, init_position=0)
    print(datetime.now())
    print("Generating Prism Program...")
    generate_modules('result.txt', total_seconds=500, laps=30, track_definition=track_def, car_definitions=[car_def1], time_precision=TimePrecision.Tenths, is_game=True)
    formula_str = "R{\"total_time\"}min=? [F \"goal\"]"
    print("Generated Prism Program")
    print(datetime.now())
    if RUN_STORMPY:
        import stormpy
        print("parsing Prism program with Storm...")
        program = stormpy.parse_prism_program('result2.txt')
        print("parsed program")
        print(datetime.now())
        print("parsing formulas...")
        formulas = stormpy.parse_properties_for_prism_program(formula_str, program)
        print("parsed formulas")
        print(datetime.now())
        print("building model...")
        model = stormpy.build_model(program, formulas)
        print(model)
        initial_state = model.initial_states[0]
        print("built model")
        print(datetime.now())
        print("Checking model...")
        result = stormpy.model_checking(model, formulas[0], extract_scheduler=True)
        print("Finished checking")
        print(datetime.now())
        print(result.at(initial_state))