
scenario_type = {'two_player_smg1': 'Straight', 'two_player_smg2': 'Hairpin', 'two_player_smg3': 'Chicane'}
all_results = []
with open('all_scenarios_log.txt', 'r') as logfile:
    while True:
        line = logfile.readline()
        if line.startswith('Parsing model file'):
            # Process Scenario type
            pass
        elif line.startswith('Building model...'):
            # Process constants
            line = logfile.readline()
            pass
        elif line.startswith('States:'):
            # Process Model Size
            pass
        elif line.startswith('Time for model construction:'):
            # Process build time
            pass
        elif line.startswith('Time for model checking:'):
            # Process computation time
            pass
        elif line.startswith('Result:'):
            # Process result
            pass