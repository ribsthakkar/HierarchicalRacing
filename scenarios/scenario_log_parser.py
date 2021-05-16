import pandas as pd

scenario_type = {'two_player_smg1': 'Straight', 'two_player_smg2': 'Hairpin', 'two_player_smg3': 'Chicane'}
all_results = []
with open('all_scenarios_log.txt', 'r') as logfile:
    while True:
        line = logfile.readline()
        if not line: break
        if line.startswith('Parsing model file'):
            # Process Scenario type
            data = {}
            for t in scenario_type:
                if t in line:
                    data['type'] = scenario_type[t]
                    break
        elif line.startswith('Building model...'):
            # Process constants
            line = logfile.readline()
            const_str = line.split(' ')[2]
            constants = const_str.split(',')
            for c in constants:
                lhs, rhs = c.split('=')
                data[lhs] = int(rhs)
        elif line.startswith('States:'):
            # Process Model Size
            states = line.split(' ')[6]
            data['num_states'] = int(states)
            pass
        elif line.startswith('Time for model construction: '):
            # Process build time
            removed = line.replace('Time for model construction: ', '')
            data['construction_time'] = float(removed.split(' ')[0])
            pass
        elif line.startswith('Time for model checking: '):
            # Process computation time
            removed = line.replace('Time for model checking: ', '')
            data['checking_time'] = float(removed.split(' ')[0])
            data['total_time'] = data['checking_time'] + data['construction_time']
            pass
        elif line.startswith('Result:'):
            # Process result
            removed = line.replace('Result: ', '')
            res = int(float(removed.split(' ')[0]))
            data['time_gap'] = res - data['max_time']
            print(data)
            all_results.append(data)
            pass

df = pd.DataFrame(all_results)
df[df['type']=='Straight'].to_excel('straight.xlsx')
df[df['type']=='Hairpin'].to_excel('hairpin.xlsx')
df[df['type']=='Chicane'].to_excel('chicane.xlsx')
