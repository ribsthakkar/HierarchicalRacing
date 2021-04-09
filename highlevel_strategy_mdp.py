from typing import Dict, List, TextIO
from example import map_sequences_to_probabilities, get_sorted_weather_sequences
from weather_planning_with_storm import generate_weather_sequences
from constants import *

def _write_with_newline_and_sc(line:str, output: TextIO, include_semicolon=True):
    output.write(line + ';\n' if include_semicolon else line + '\n')

def get_valid_directions(x_size, y_size, x, y):
    valid_directions = {"n", "ne", "e", "se", "s", "sw", "w", "nw"}
    if x == 1:
        valid_directions.discard("sw")
        valid_directions.discard("w")
        valid_directions.discard("nw")
    if y == 1:
        valid_directions.discard("s")
        valid_directions.discard("se")
        valid_directions.discard("sw")
    if x == x_size:
        valid_directions.discard("ne")
        valid_directions.discard("se")
        valid_directions.discard("e")
    if y == y_size:
        valid_directions.discard("n")
        valid_directions.discard("ne")
        valid_directions.discard("nw")
    return list(valid_directions)

def get_weather_affected_directions(agent_dir, x_size, y_size, x, y):
    weather_affected_directions = {"n":{"e", "w", "nw", "ne"}, "ne":{"n", "e"}, "e":{"n", "s", "ne", "se"},
                        "se":{"s", "e"}, "s":{"e", "w", "sw", "se"}, "sw":{"s", "w"}, "w":{"n", "s", "nw", "sw"},
                        "nw":{"n", "w"}}

    weather_directions = weather_affected_directions[agent_dir]
    if y == 1 and agent_dir == 'e':
        weather_directions.discard('s')
        weather_directions.discard('se')
    if y == 1 and agent_dir == 'w':
        weather_directions.discard('s')
        weather_directions.discard('sw')
    if x == 1 and agent_dir == 'n':
        weather_directions.discard('w')
        weather_directions.discard('nw')
    if x == 1 and agent_dir == 's':
        weather_directions.discard('w')
        weather_directions.discard('sw')

    if y == y_size and agent_dir == 'e':
        weather_directions.discard('n')
        weather_directions.discard('ne')
    if y == y_size and agent_dir == 'w':
        weather_directions.discard('n')
        weather_directions.discard('nw')
    if x == x_size and agent_dir == 'n':
        weather_directions.discard('e')
        weather_directions.discard('ne')
    if x == x_size and agent_dir == 's':
        weather_directions.discard('e')
        weather_directions.discard('se')
    return weather_directions

def get_target_coords(dir, x, y):
    target_x = x
    target_y = y
    if 's' in dir:
        target_y -= 1
    elif 'n' in dir:
        target_y += 1
    if 'w' in dir:
        target_x -= 1
    elif 'e' in dir:
        target_x += 1

    return target_x, target_y

class TrackStraight:
    def __init__(self, length):
        pass

class TrackCorner:
    class Entry:
        pass
    class Mid:
        pass
    class Exit:
        pass
    def __init__(self, radius, ):
        pass

class TrackDef:
    def __init__(self,  track_landmarks, num_laps = 1, num_lines=3):
        self.landmarks = self._process_landmarks(track_landmarks)
        self.track_points= len(self.landmarks)
        self.tls = num_lines
        pass

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


    def _ts2te(self, position, line, tp_update_string, car_def, straight, entry):
        output = []
        for ta in range(0, 100, 10):
            for tl in range(self.tls):
                if tl == line: continue
                guard = f"track_pos={position} & track_line={tl} & {ta}<=tire_age & tire_age<{ta+10}"
                expected_time = straight.length/2

    def generate_guards_and_updates(self, position, line, tp_update_string, car_definition):
        tail = self.landmarks[position % self.tls]
        head = self.landmarks[(position+1) % self.tls]
        if type(tail) == TrackStraight and type(head) == TrackCorner.Entry:
            return self._ts2te(position, line, tp_update_string, car_definition, tail, head)
            pass
        elif type(tail) == TrackCorner.Entry and type(head) == TrackCorner.Mid:
            pass
        elif type(tail) == TrackCorner.Mid and type(head) == TrackCorner.Exit:
            pass
        elif type(tail) == TrackCorner.Exit and type(head) == TrackStraight:
            pass
        elif type(tail) == TrackCorner.Exit and type(head) == TrackCorner.Entry:
            pass
        elif type(tail) == TrackStraight and type(head) == TrackStraight:
            pass
        else:
            print("Invalid Connection")
            exit(1)

def generate_racecar_module(output_file, id, max_time, track_definition, car_definition):
    with open(output_file, "w+") as output:
        _write_with_newline_and_sc("mdp", output, False)

        tps = track_definition.track_points
        tls = track_definition.track_lines
        max_v = car_definition.max_velocity
        init_tire = car_definition.tire_age
        init_time = car_definition.init_time
        init_line = car_definition.init_line
        init_v = car_definition.init_velocity
        init_pos = car_definition.init_position
        _write_with_newline_and_sc(f'module racecar{id}\n', output, False)
        _write_with_newline_and_sc(f't : [0..{max_time}] init {init_time}', output)
        _write_with_newline_and_sc(f'track_pos : [0..{tps}] init {init_pos}', output)
        _write_with_newline_and_sc(f'track_line : [0..{tls}] init {init_line}', output)
        _write_with_newline_and_sc(f'tire_age : [0..100] init {init_tire}', output)
        _write_with_newline_and_sc(f'velocity : [0..{max_v}] init {init_v}', output)
        _write_with_newline_and_sc("", output, False)

        tp_update_string = "(track_pos=track_pos'+1)"
        for track_pos in range(init_pos, tps):
            for line in range(0, tls):
                action = f"step{track_pos}_{line}"
                for tire_age_guard, updates in track_definition.generate_guards_and_updates(track_pos, line, tp_update_string, car_definition):
                    guard = f"track_pos={track_pos} & {tire_age_guard}"
                    _write_with_newline_and_sc(f"{action} {guard} -> {'+'.join(map(lambda prob, update: f'{prob}:{update}', updates))}", output)



        _write_with_newline_and_sc("endmodule", output, False)
        _write_with_newline_and_sc("", output, False)

if __name__ == "__main__":

    all_possible_sequences = get_sorted_weather_sequences(weather_horizon)
    weather_distributions = map_sequences_to_probabilities(all_possible_sequences)
    weather_sequences = generate_weather_sequences(x_states, y_states, time_horizon, weather_horizon)
    # print(weather_sequences)
    generate_prism_robot_module_deterministic(x_states, y_states, x_init, y_init, x_goal, y_goal, weather_sequences, weather_distributions, 'result.txt', time_horizon, weather_horizon)