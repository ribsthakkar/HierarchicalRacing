import math
from datetime import datetime
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
        iv = u + a_max*x[0]
        t2 = (v-iv)/-abs(a_min)
        return x[1] + t2

    def time_variables_constraint(x):
        return x[1] - x[0]
    nlc = NonlinearConstraint(time_variables_constraint, 0, np.inf)
    constraints.append(nlc)

    def distance_constraint(x):
        iv = u + a_max*x[0]
        dist = (u+iv)*x[0]/2
        dist += (x[1]-x[0])*iv
        t2 = (v-iv)/-abs(a_min)
        dist += (iv + v)*t2
        return dist
    nlc = NonlinearConstraint(distance_constraint, s, s)
    constraints.append(nlc)

    def max_vel_constraint(x):
        iv = u+ a_max*x[0]
        return iv
    nlc = NonlinearConstraint(max_vel_constraint, 0, v_max)
    constraints.append(nlc)

    lb = [0, 0]
    ub = [100, 100]
    bounds = Bounds(lb, ub)
    x0 = [0,3]
    result = minimize(opt, x0, constraints=constraints, bounds=bounds)
    if (is_greater_than(max_vel_constraint(result.x), v_max) or not math.isclose(distance_constraint(result.x), s) or result.x[0] <= -1e-5 or result.x[1] <= 1e-5):
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
    def __init__(self, track_landmarks, width, pit_exit_position, pit_exit_velocity, pit_exit_line, pit_time, num_lines=3):
        self.landmarks = self._process_landmarks(track_landmarks)
        self.track_points= len(self.landmarks)
        self.track_lines = num_lines
        self.width = width
        self.pit_exit_p = pit_exit_position
        self.pit_exit_l = pit_exit_line
        self.pit_exit_v = pit_exit_velocity
        self.pit_time = pit_time

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


    def generate_guards_and_updates(self, idx, position, line, velocity, car_def, max_time, mdp_tire_age_limit):
        tail = self.landmarks[position % self.track_points]
        head = self.landmarks[(position+1) % self.track_points]
        output = []
        tire_wear_step = 10
        velocity_step = car_def.velocity_step
        for tl in range(self.track_lines):
            for ta in range(0, MAX_TIRE_AGE, tire_wear_step):
                for init_v in range(1, car_def.max_v+1, velocity_step):
                    updates = []
                    avg_init_v = min(init_v + velocity_step / 2, car_def.max_v)
                    avg_ta = ta + tire_wear_step / 2
                    dist = math.sqrt(
                        (tail.length) ** 2 + (abs(tl - line) * (self.width) / (self.track_lines + 1)) ** 2)
                    feasible_v = {}
                    for target_v in range(velocity, min(car_def.max_v+1, velocity+velocity_step)):
                        if not tail.is_v_feasible(target_v, tl, avg_ta, car_def.min_gs, car_def.max_gs): break
                        min_time = calc_min_time(avg_init_v, target_v, dist,
                                                 car_def.max_braking, car_def.max_acceleration, car_def.max_v)
                        if min_time is None:continue
                        feasible_v[target_v] = min_time
                    max_dt = 0
                    max_dta = 0
                    for target_v, min_time in feasible_v.items():
                        est_dt = int(math.ceil(min_time))
                        max_dt = max(est_dt, max_dt)
                        est_dta = int(tail.tire_wear(target_v, tl, car_def.tire_wear_factor))
                        max_dta = max(max_dta, est_dta)
                        prob = f'1/{len(feasible_v)}'
                        changes = f"(velocity{idx}'={target_v}) & (track_line{idx}'={line}) & (tire_age{idx}'=tire_age{idx}+{est_dta}) & (t{idx}'=t{idx}+{est_dt})"
                        updates.append((prob, changes))
                    guard = f"track_line{idx}={tl} & {ta}<=tire_age{idx} & tire_age{idx}<{ta + tire_wear_step} & {init_v}<=velocity{idx} & velocity{idx}<{init_v + velocity_step} & t{idx}<={max_time - max_dt} & tire_age{idx}<={mdp_tire_age_limit - max_dta}"
                    if len(updates):
                        output.append((guard, updates))
        updates = []
        est_dt = int(head.length / 1)
        prob = '1'
        changes = f"(velocity{idx}'={1}) & (track_line{idx}'={line}) & (tire_age{idx}'=tire_age{idx}) & (t{idx}'=t{idx}+{est_dt})"
        guard = f"tire_age{idx} >={MAX_TIRE_AGE} & t{idx}<={max_time - est_dt}"
        updates.append((prob, changes))
        output.append((guard, updates))
        return output


def generate_modules(output_file, max_time, laps, track_definition, car_definitions):
    with open(output_file, "w+") as output:
        _write_with_newline_and_sc("mdp", output, False)
        tps = track_definition.track_points
        tls = track_definition.track_lines
        pit_out = track_definition.pit_exit_p
        for idx, car_definition in enumerate(car_definitions):
            max_v = car_definition.max_v
            init_tire = car_definition.init_tire
            init_time = car_definition.init_time
            init_line = car_definition.init_line
            init_v = car_definition.init_velocity
            init_pos = car_definition.init_position

            # Position/Lap Module
            _write_with_newline_and_sc(f'module progress{idx}\n', output, False)
            _write_with_newline_and_sc(f'track_pos{idx} : [0..{tps-1}] init {init_pos}', output)
            _write_with_newline_and_sc(f'lap{idx} : [0..{laps}] init 0', output)
            _write_with_newline_and_sc(f"[] track_pos{idx} < {tps-1} & lap{idx} < {laps} ->  1: (track_pos{idx}'=track_pos{idx}+1)", output)
            _write_with_newline_and_sc(f"[] track_pos{idx} = {tps-1} & lap{idx} < {laps} ->  1: (track_pos{idx}'=0) & (lap{idx}' = lap{idx}+1)", output)
            _write_with_newline_and_sc(f"[pit_{idx}] track_pos{idx} = {tps-1} & lap{idx} < {laps} -> 1: (track_pos{idx}'={pit_out}) & (lap{idx}' = lap{idx}+1)", output)
            _write_with_newline_and_sc("endmodule", output, False)
            _write_with_newline_and_sc(f"formula car_{idx}_position = track_pos{idx}", output)

            # Tire Age Module
            _write_with_newline_and_sc(f'module racecar{idx}\n', output, False)
            _write_with_newline_and_sc(f't{idx} : [0..{max_time}] init {init_time}', output)
            _write_with_newline_and_sc(f'track_line{idx} : [0..{max(1, tls-1)}] init {init_line}', output)
            _write_with_newline_and_sc(f'tire_age{idx} : [0..{MAX_TIRE_AGE+100}] init {init_tire}', output)
            _write_with_newline_and_sc(f'velocity{idx} : [1..{max_v}] init {init_v}', output)
            _write_with_newline_and_sc("", output, False)
            for track_pos in range(0, tps):
                for line in range(0, tls):
                    for velocity in range(1, max_v+1, car_definition.velocity_step):
                        action = f"[step{track_pos}_{line}_{velocity}_{velocity+car_definition.velocity_step}_{idx}]"
                        for ta_v_guard, updates in track_definition.generate_guards_and_updates(idx, track_pos, line, velocity, car_definition, max_time, MAX_TIRE_AGE+100):
                            if len(updates) == 0: continue
                            guard = f"car_{idx}_position={track_pos} & {ta_v_guard}"
                            _write_with_newline_and_sc(f"{action} {guard} -> {'+'.join(map(lambda pair: f'{pair[0]}:{pair[1]}', updates))}", output)

            action = f"[pit_{idx}]"
            guard = f"car_{idx}_position={tps-1} & t{idx}<{max_time-track_def.pit_time}"
            _write_with_newline_and_sc(
                f"{action} {guard} -> 1:(tire_age{idx}'=0) & (velocity{idx}'={min(track_def.pit_exit_v, max_v)}) & (track_line{idx}'={track_def.pit_exit_l}) & (t{idx}'=t{idx}+{track_def.pit_time})", output)

            _write_with_newline_and_sc("endmodule", output, False)
            _write_with_newline_and_sc("", output, False)
            _write_with_newline_and_sc(f'label \"goal{idx}\" = (lap{idx}={laps} & track_pos{idx}=0) | (lap{idx}={laps} & track_pos{idx}={track_def.pit_exit_p})', output)

            _write_with_newline_and_sc(f"rewards \"total_time{idx}\"", output, False)
            _write_with_newline_and_sc(f"lap{idx}={laps-1} & track_pos{idx}={tps-1}: t{idx}", output)
            _write_with_newline_and_sc("endrewards", output, False)

if __name__ == "__main__":
    NUM_LINES = 2
    WIDTH = 5
    components = [TrackStraight(5), TrackCorner(num_lines=NUM_LINES, width=WIDTH, inside_turn_radius=2.5, degrees=90, exit_length=2.3),
                  TrackCorner(num_lines=NUM_LINES, width=WIDTH, inside_turn_radius=2.5, degrees=90, exit_length=5.0), TrackStraight(5.0),
                  TrackCorner(num_lines=NUM_LINES, width=WIDTH, inside_turn_radius=2.5, degrees=90, exit_length=2.3),
                  TrackCorner(num_lines=NUM_LINES, width=WIDTH, inside_turn_radius=2.5, degrees=90, exit_length=5.0)]
    # components = [TrackStraight(50), TrackCorner(num_lines=NUM_LINES, width=WIDTH, inside_turn_radius=25, degrees=30, exit_length=23)]
    pit_exit_p = 1
    pit_exit_v = 3
    pit_exit_line = 0
    pit_time = 20
    track_def = TrackDef(components, width=WIDTH, pit_exit_position=pit_exit_p, pit_exit_velocity=pit_exit_v, pit_exit_line=pit_exit_line, pit_time=pit_time, num_lines=NUM_LINES)
    car_def1 = CarDef(max_velocity=10, velocity_step=2, min_gs=0.1*9.8, max_gs=0.3*9.8, max_braking=2, max_acceleration=2,
                     tire_wear_factor=.7,
                     init_time=0, init_tire=0, init_line=0, init_velocity=3, init_position=0)
    car_def2 = CarDef(max_velocity=10, velocity_step=2, min_gs=0.1*9.8, max_gs=0.3*9.8, max_braking=2, max_acceleration=2,
                     tire_wear_factor=1,
                     init_time=0, init_tire=0, init_line=1, init_velocity=3, init_position=0)
    print(datetime.now())
    print("Generating Prism Program...")
    generate_modules('result.txt', max_time=250, laps=30, track_definition=track_def, car_definitions=[car_def1])
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