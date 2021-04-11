import math
from typing import Dict, List, TextIO

from util import _write_with_newline_and_sc, neg_to_str
import stormpy

class CarDef:
    def __init__(self, max_velocity, min_gs, max_gs, max_braking, max_acceleration, init_tire, init_time, init_line, init_velocity, init_position):
        self.max_v = max_velocity
        self.max_braking = -abs(max_braking)
        self.max_acceleration = max_acceleration
        self.min_gs = min_gs
        self.max_gs = max_gs
        self.init_tire = init_tire
        self.init_time = init_time
        self.init_line = init_line
        self.init_velocity = init_velocity
        self.init_position = init_position
        pass

class TrackComponent:
    def __init__(self, length):
        self.length = length

    def is_v_feasible(self, velocity, tire_wear, min_cornering_gs, max_cornering_gs):
        raise NotImplementedError("Must be implemented by child classes")

    def tire_wear(self, velocity):
        raise NotImplementedError("Must be implemented by child classes")


class TrackStraight(TrackComponent):
    def __init__(self, length):
        super().__init__(length)

    def is_v_feasible(self, velocity, tire_wear, min_cornering_gs, max_cornering_gs):
        return True

    def tire_wear(self, velocity):
        if velocity == 0: return 0
        return 1

class TrackCorner:
    class Entry(TrackComponent):
        def __init__(self, length, turn_radius):
            super().__init__(length)
            self.tr = turn_radius

        def is_v_feasible(self, velocity, tire_wear, min_cornering_gs, max_cornering_gs):
            gs = (velocity**2)/self.tr
            g_diff = (max_cornering_gs-min_cornering_gs)*(tire_wear/1000)
            return gs <= max_cornering_gs-g_diff

        def tire_wear(self, velocity):
            gs = (velocity**2)/self.tr
            return int((math.ceil(gs) * self.length/velocity)/10)


    class Mid(TrackComponent):
        def __init__(self, length, turn_radius):
            super().__init__(length)
            self.tr = turn_radius

        def is_v_feasible(self, velocity, tire_wear, min_cornering_gs, max_cornering_gs):
            gs = (velocity**2)/self.tr
            g_diff = (max_cornering_gs-min_cornering_gs)*(tire_wear/1000)
            return gs <= max_cornering_gs-g_diff

        def tire_wear(self, velocity):
            gs = (velocity**2)/self.tr
            return int((math.ceil(gs) * self.length/velocity)/10)


    def __init__(self, turn_radius, entry_length, mid_length, exit_length):
        self.entry = self.Entry(entry_length, turn_radius)
        self.mid = self.Mid(mid_length, turn_radius)
        self.exit = TrackStraight(exit_length)


class TrackDef:
    def __init__(self, track_landmarks, width, pit_exit_p, pit_exit_v, pit_exit_l, pit_time, num_lines=3):
        self.landmarks = self._process_landmarks(track_landmarks)
        self.track_points= len(self.landmarks)
        self.track_lines = num_lines
        self.width = width
        self.pit_exit_p = pit_exit_p
        self.pit_exit_l = pit_exit_l
        self.pit_exit_v = pit_exit_v
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


    def _ts2te(self, line, acc, tp_update_string, car_def, straight, entry, lap_change):
        lap_change_update = "" if not lap_change else "& (lap'=lap+1)"
        # lap_change_update = ""
        output = []
        tire_wear_step = 100
        for tl in range(self.track_lines):
            for ta in range(0, 1000, tire_wear_step):
                for v in range(0, car_def.max_v, 10):
                    updates = []
                    guard = f"track_line={tl} & {ta}<=tire_age & tire_age<{ta+tire_wear_step} & {v}<=velocity & velocity<{v+10}"
                    avg_v = v+10/2
                    avg_ta = ta+tire_wear_step/2
                    avg_dist = math.sqrt((straight.length/2)**2 + (abs(tl - line)*(self.width)/(self.track_lines+1))**2)
                    avg_final_v = (math.sqrt(max(0, avg_v**2 + 2*acc*avg_dist)))
                    max_fv = min(car_def.max_v, int(avg_final_v + 4))
                    min_fv = max(1, int(avg_final_v - 4))
                    feasible_v = []
                    for fv in range(min_fv, max_fv+1):
                        if entry.is_v_feasible(fv, avg_ta, car_def.min_gs, car_def.max_gs):
                            feasible_v.append(fv)
                    for fv in feasible_v:
                        est_dt = int((2 * avg_dist / (avg_v + fv)))
                        est_dta = entry.tire_wear(fv)
                        prob = f'1/{len(feasible_v)}'
                        changes = f"{tp_update_string} & (velocity'={fv}) & (track_line'={line}) & (tire_age'=tire_age+{est_dta}) & (t'=t+{est_dt}) {lap_change_update}"
                        updates.append((prob, changes))

                    if len(updates):
                        output.append((guard, updates))
                if acc <= 0:
                    updates = []
                    guard = f"track_line={tl} & {ta}<=tire_age & tire_age<{ta + 10} & velocity={car_def.max_v}"
                    avg_v = car_def.max_v
                    avg_ta = ta + tire_wear_step / 2
                    avg_dist = math.sqrt(
                        (straight.length / 2) ** 2 + (abs(tl - line) * (self.width) / (self.track_lines + 1)) ** 2)
                    avg_final_v = (math.sqrt(avg_v ** 2 + 2 * acc * avg_dist))
                    max_fv = min(car_def.max_v, int(avg_final_v + 4))
                    min_fv = max(0, int(avg_final_v - 4))
                    feasible_v = []
                    for fv in range(min_fv, max_fv + 1):
                        if entry.is_v_feasible(fv, avg_ta, car_def.min_gs, car_def.max_gs):
                            feasible_v.append(fv)
                    for fv in feasible_v:
                        est_dt = int((2 * avg_dist / (avg_v + fv)))
                        est_dta = entry.tire_wear(fv)
                        prob = f'1/{len(feasible_v)}'
                        changes = f"{tp_update_string} & (velocity'={fv}) & (track_line'={line}) & (tire_age'=tire_age+{est_dta}) & (t'=t+{est_dt}) {lap_change_update}"
                        updates.append((prob, changes))
                    if len(updates):
                        output.append((guard, updates))
            updates = []
            guard = f"tire_age >=1000"
            est_dt = int(straight.length / 1)
            prob = '1'
            changes = f"{tp_update_string} & (velocity'={1}) & (track_line'={line}) & (tire_age'=tire_age) & (t'=t+{est_dt}) {lap_change_update}"
            updates.append((prob, changes))
            output.append((guard, updates))
        return output

    def generate_guards_and_updates(self, position, line, acc, tp_update_string, car_definition, lap_change):
        tail = self.landmarks[position % self.track_points]
        head = self.landmarks[(position+1) % self.track_points]
        return self._ts2te(line, acc, tp_update_string, car_definition, tail, head, lap_change)
        # if type(tail) == TrackStraight and type(head) == TrackCorner.Entry:
        #     return self._ts2te(line, acc, tp_update_string, car_definition, tail, head)
        #     pass
        # elif type(tail) == TrackCorner.Entry and type(head) == TrackCorner.Mid:
        #     pass
        # elif type(tail) == TrackCorner.Mid and type(head) == TrackCorner.Exit:
        #     pass
        # elif type(tail) == TrackCorner.Exit and type(head) == TrackStraight:
        #     pass
        # elif type(tail) == TrackCorner.Exit and type(head) == TrackCorner.Entry:
        #     pass
        # elif type(tail) == TrackStraight and type(head) == TrackStraight:
        #     pass
        # else:
        #     print("Invalid Connection")
        #     exit(1)

def generate_racecar_module(output_file, id, max_time, laps, track_definition, car_definition):
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
        _write_with_newline_and_sc(f'module racecar{id}\n', output, False)
        _write_with_newline_and_sc(f't : [0..{max_time}] init {init_time}', output)
        _write_with_newline_and_sc(f'lap : [0..{laps}] init 0', output)
        _write_with_newline_and_sc(f'track_pos : [0..{tps-1}] init {init_pos}', output)
        _write_with_newline_and_sc(f'track_line : [0..{tls-1}] init {init_line}', output)
        _write_with_newline_and_sc(f'tire_age : [0..1000] init {init_tire}', output)
        _write_with_newline_and_sc(f'velocity : [1..{max_v}] init {init_v}', output)
        _write_with_newline_and_sc("", output, False)

        for track_pos in range(0, tps):
            lap_change = track_pos==tps-1
            tp_update_string = "(track_pos'=track_pos+1)" if not lap_change  else "(track_pos'=0)"
            for line in range(0, tls):
                for acc in range(car_definition.max_braking, car_definition.max_acceleration):
                    action = f"[step{track_pos}_{line}_{neg_to_str(acc)}]"
                    for ta_v_guard, updates in track_definition.generate_guards_and_updates(track_pos, line, acc, tp_update_string, car_definition, lap_change):
                        if len(updates) == 0: continue
                        guard = f"track_pos={track_pos} & {ta_v_guard} & lap<{laps}"
                        _write_with_newline_and_sc(f"{action} {guard} -> {'+'.join(map(lambda pair: f'{pair[0]}:{pair[1]}', updates))}", output)

        tp_update_string = "(track_pos'=0)"
        action = f"[pit]"
        guard = f"track_pos={tps-1} & lap<{laps}"
        _write_with_newline_and_sc(
            f"{action} {guard} -> 1:(track_pos'={track_def.pit_exit_p}) + (tire_age'=0) & (velocity'={min(track_def.pit_exit_v, max_v)}) & (track_line'={track_def.pit_exit_l}) & (t'=t+{track_def.pit_time}) & (lap'=lap+1)", output)

        _write_with_newline_and_sc("endmodule", output, False)
        _write_with_newline_and_sc("", output, False)
        _write_with_newline_and_sc(f'label \"goal\" = lap={laps}', output)

        _write_with_newline_and_sc("rewards \"total_time\"", output, False)
        _write_with_newline_and_sc(f"true: t", output)
        _write_with_newline_and_sc("endrewards", output, False)

if __name__ == "__main__":
    components = [TrackStraight(500), TrackCorner(250, 390, 390, 226), TrackCorner(250, 390, 390, 500), TrackStraight(500), TrackCorner(250,390, 390,226), TrackCorner(250, 390, 390, 500)]
    components = [TrackStraight(500), TrackCorner(250, 390, 390, 226)]
    pit_exit_p = 1
    pit_exit_v = 30
    pit_exit_line = 0
    pit_time = 30
    track_def = TrackDef(components, 20, pit_exit_p, pit_exit_v, pit_exit_line, pit_time)
    car_def = CarDef(100, 9.8, 3*9.8, 10, 10, 0, 0, 1, 10, 0)
    generate_racecar_module('result2.txt', id=1, max_time=1000, laps=1, track_definition=track_def, car_definition=car_def)
    formula_str = "R{\"total_time\"}min=? [F \"goal\"]"

    print("parsing program...")
    program = stormpy.parse_prism_program('result2.txt')
    print("parsed program")
    print("parsing formulas...")
    formulas = stormpy.parse_properties_for_prism_program(formula_str, program)
    print("parsed formulas")
    print("building model...")
    model = stormpy.build_model(program, formulas)
    print("built model")
    print("Checking model...")
    result = stormpy.model_checking(model, formulas[0], extract_scheduler=False)
    print("Finished checking")
    print(result)