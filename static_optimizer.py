import math
from collections import deque


def static_race_optimize(agent, opponent_cars, replan_time, input_update_time):
    actions = deque()
    t = 0
    while t <= (replan_time) + input_update_time / 2:
        actions.append((5, math.radians(10), (5, math.pi)))
        t += input_update_time

    return actions