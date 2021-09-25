import math
from datetime import datetime
from typing import Dict, List, TextIO

from util import _write_with_newline_and_sc, neg_to_str
MAX_TIRE_AGE = 100
RUN_STORMPY = False
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
        def __init__(self, length, turn_radius, width, num_lines):
            super().__init__(length)
            self.tr = list(map(lambda l: turn_radius + l*width/num_lines, range(num_lines)))

        def is_v_feasible(self, velocity, line, tire_wear, min_cornering_gs, max_cornering_gs):
            gs = (velocity**2)/self.tr[line]
            g_diff = (max_cornering_gs-min_cornering_gs)*(tire_wear/MAX_TIRE_AGE)
            return gs <= max_cornering_gs-g_diff

        def tire_wear(self, velocity, line, tire_wear_factor):
            gs = (velocity**2)/self.tr[line]
            return int((math.ceil(gs) * self.length/velocity)/tire_wear_factor)


    class Mid(TrackComponent):
        def __init__(self, length, turn_radius, width, num_lines):
            super().__init__(length)
            self.tr = list(map(lambda l: turn_radius + l*width/num_lines, range(num_lines)))

        def is_v_feasible(self, velocity, line, tire_wear, min_cornering_gs, max_cornering_gs):
            gs = (velocity**2)/self.tr[line]
            g_diff = (max_cornering_gs-min_cornering_gs)*(tire_wear/MAX_TIRE_AGE)
            return gs <= max_cornering_gs-g_diff

        def tire_wear(self, velocity, line, tire_wear_factor):
            gs = (velocity**2)/self.tr[line]
            return int((math.ceil(gs) * self.length/velocity)/tire_wear_factor)


    def __init__(self, num_lines, width, inside_turn_radius, entry_length, mid_length, exit_length):
        self.entry = self.Entry(entry_length, inside_turn_radius, width, num_lines)
        self.mid = self.Mid(mid_length, inside_turn_radius, width, num_lines)
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


    def _ts2te(self, line, acc, tp_update_string, car_def, straight, entry, max_time, mdp_tire_age_limit,  lap_change):
        lap_change_update = "" if not lap_change else "& (lap'=lap+1)"
        # lap_change_update = ""
        output = []
        tire_wear_step = 10
        velocity_step = car_def.velocity_step
        for tl in range(self.track_lines):
            for ta in range(0, MAX_TIRE_AGE, tire_wear_step):
                for v in range(1, car_def.max_v, velocity_step):
                    updates = []
                    avg_v = v+velocity_step/2
                    avg_ta = ta+tire_wear_step/2
                    avg_dist = math.sqrt((straight.length)**2 + (abs(tl - line)*(self.width)/(self.track_lines+1))**2)
                    try:
                        avg_final_v = avg_v + (-avg_v+math.sqrt(avg_v**2 + 2*acc*avg_dist))
                    except:
                        print(avg_v, acc)
                        continue
                    max_fv = min(car_def.max_v, int(avg_final_v + (velocity_step)))
                    min_fv = max(1, int(avg_final_v - (velocity_step)))
                    print(min_fv, max_fv, acc)
                    feasible_v = []
                    for fv in range(min_fv, max_fv+1):
                        if entry.is_v_feasible(fv, tl, avg_ta, car_def.min_gs, car_def.max_gs):
                            feasible_v.append(fv)
                    max_dt = 0
                    max_dta = 0
                    for fv in feasible_v:
                        est_dt = int((2 * avg_dist / (avg_v + fv)))
                        max_dt = max(est_dt, max_dt)
                        est_dta = entry.tire_wear(fv, tl, car_def.tire_wear_factor)
                        max_dta = max(max_dt, max_dta)
                        prob = f'1/{len(feasible_v)}'
                        changes = f"{tp_update_string} & (velocity'={fv}) & (track_line'={line}) & (tire_age'=tire_age+{est_dta}) & (t'=t+{est_dt}) {lap_change_update}"
                        updates.append((prob, changes))
                    guard = f"track_line={tl} & {ta}<=tire_age & tire_age<{ta+tire_wear_step} & {v}<=velocity & velocity<{v+velocity_step} & t<={max_time-max_dt} & tire_age<={mdp_tire_age_limit-max_dta}"
                    if len(updates):
                        output.append((guard, updates))
                if acc <= 0:
                    updates = []
                    avg_v = car_def.max_v
                    avg_ta = ta + tire_wear_step / 2
                    avg_dist = math.sqrt(
                        (straight.length) ** 2 + (abs(tl - line) * (self.width) / (self.track_lines + 1)) ** 2)
                    try:
                        avg_final_v = avg_v + (-avg_v+math.sqrt(avg_v**2 + 2*acc*avg_dist))*acc
                    except:
                        continue
                    max_fv = min(car_def.max_v, int(avg_final_v + (velocity_step)))
                    min_fv = max(1, int(avg_final_v - (velocity_step)))
                    feasible_v = []
                    max_dt = 0
                    max_dta = 0
                    for fv in range(min_fv, max_fv + 1):
                        if entry.is_v_feasible(fv, tl, avg_ta, car_def.min_gs, car_def.max_gs):
                            feasible_v.append(fv)
                    for fv in feasible_v:
                        est_dt = int((2 * avg_dist / (avg_v + fv)))
                        max_dt = max(est_dt, max_dt)
                        est_dta = entry.tire_wear(fv, tl, car_def.tire_wear_factor)
                        max_dta = max(max_dt, max_dta)
                        prob = f'1/{len(feasible_v)}'
                        changes = f"{tp_update_string} & (velocity'={fv}) & (track_line'={line}) & (tire_age'=tire_age+{est_dta}) & (t'=t+{est_dt}) {lap_change_update}"
                        updates.append((prob, changes))
                    guard = f"track_line={tl} & {ta}<=tire_age & tire_age<{ta+tire_wear_step} & velocity={car_def.max_v} & t<={max_time-max_dt} & tire_age<={mdp_tire_age_limit-max_dta}"
                    if len(updates):
                        output.append((guard, updates))
            updates = []
            est_dt = int(straight.length / 1)
            prob = '1'
            changes = f"{tp_update_string} & (velocity'={1}) & (track_line'={line}) & (tire_age'=tire_age) & (t'=t+{est_dt}) {lap_change_update}"
            guard = f"tire_age >={MAX_TIRE_AGE} & t<={max_time-est_dt}"
            updates.append((prob, changes))
            output.append((guard, updates))
        return output

    def generate_guards_and_updates(self, position, line, acc, tp_update_string, car_definition, max_time, mdp_tire_age_limit, lap_change):
        tail = self.landmarks[position % self.track_points]
        head = self.landmarks[(position+1) % self.track_points]
        return self._ts2te(line, acc, tp_update_string, car_definition, tail, head, max_time, mdp_tire_age_limit, lap_change)

def generate_racecar_module(output_file, max_time, laps, track_definition, car_definition):
    with open(output_file, "w+") as output:
        _write_with_newline_and_sc("mdp", output, False)

        tps = track_definition.track_points
        tls = track_definition.track_lines
        max_v = car_definition.max_v
        init_tire = car_definition.init_tire
        init_time = car_definition.init_time
        init_line = car_definition.init_line
        init_v = car_definition.init_velocity
        init_pos = car_definition.init_position
        _write_with_newline_and_sc(f'module racecar{1}\n', output, False)
        _write_with_newline_and_sc(f't : [0..{max_time}] init {init_time}', output)
        # _write_with_newline_and_sc(f'dt : [0..{max_time}] init 0', output)
        _write_with_newline_and_sc(f'lap : [0..{laps}] init 0', output)
        _write_with_newline_and_sc(f'track_pos : [0..{tps-1}] init {init_pos}', output)
        _write_with_newline_and_sc(f'track_line : [0..{min(1, tls-1)}] init {init_line}', output)
        _write_with_newline_and_sc(f'tire_age : [0..{MAX_TIRE_AGE+100}] init {init_tire}', output)
        _write_with_newline_and_sc(f'velocity : [1..{max_v}] init {init_v}', output)
        _write_with_newline_and_sc("", output, False)

        for track_pos in range(0, tps):
            lap_change = track_pos==tps-1
            tp_update_string = "(track_pos'=track_pos+1)" if not lap_change  else "(track_pos'=0)"
            for line in range(0, tls):
                for acc in range(car_definition.max_braking, car_definition.max_acceleration):
                    action = f"[step{track_pos}_{line}_{neg_to_str(acc)}]"
                    for ta_v_guard, updates in track_definition.generate_guards_and_updates(track_pos, line, acc, tp_update_string, car_definition, max_time, MAX_TIRE_AGE+100, lap_change):
                        if len(updates) == 0: continue
                        guard = f"track_pos={track_pos} & {ta_v_guard} & lap<{laps}"
                        _write_with_newline_and_sc(f"{action} {guard} -> {'+'.join(map(lambda pair: f'{pair[0]}:{pair[1]}', updates))}", output)

        action = f"[pit]"
        guard = f"track_pos={tps-1} & lap<{laps} & t <{max_time-track_def.pit_time}"
        _write_with_newline_and_sc(
            f"{action} {guard} -> 1:(track_pos'={track_def.pit_exit_p}) & (tire_age'=0) & (velocity'={min(track_def.pit_exit_v, max_v)}) & (track_line'={track_def.pit_exit_l}) & (t'=t+{track_def.pit_time}) & (lap'=lap+1)", output)

        _write_with_newline_and_sc("endmodule", output, False)
        _write_with_newline_and_sc("", output, False)
        _write_with_newline_and_sc(f'label \"goal\" = (lap={laps} & track_pos=0) | (lap={laps} & track_pos={track_def.pit_exit_p})', output)

        _write_with_newline_and_sc("rewards \"total_time\"", output, False)
        _write_with_newline_and_sc(f"lap={laps-1} & track_pos={tps-1}: t", output)
        _write_with_newline_and_sc("endrewards", output, False)

if __name__ == "__main__":
    NUM_LINES = 1
    WIDTH = 5
    components = [TrackStraight(50), TrackCorner(NUM_LINES, WIDTH, 25, 39, 39, 23), TrackCorner(NUM_LINES, WIDTH, 25, 39, 39, 50), TrackStraight(50), TrackCorner(NUM_LINES, WIDTH, 25, 39, 39,23), TrackCorner(NUM_LINES, WIDTH, 25, 39, 39, 50)]
    # components = [TrackStraight(50), TrackCorner(NUM_LINES, WIDTH, 25, 39, 39, 23)]
    pit_exit_p = 1
    pit_exit_v = 3
    pit_exit_line = 0
    pit_time = 20
    track_def = TrackDef(components, width=WIDTH, pit_exit_position=pit_exit_p, pit_exit_velocity=pit_exit_v, pit_exit_line=pit_exit_line, pit_time=pit_time, num_lines=NUM_LINES)
    car_def = CarDef(max_velocity=10, velocity_step=1, min_gs=0.3*9.8, max_gs=1*9.8, max_braking=5, max_acceleration=5,
                     tire_wear_factor=1,
                     init_time=0, init_tire=0, init_line=0, init_velocity=3, init_position=0)
    print(datetime.now())
    print("Generating Prism Program...")
    generate_racecar_module('result.txt', max_time=150, laps=3, track_definition=track_def, car_definition=car_def)
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