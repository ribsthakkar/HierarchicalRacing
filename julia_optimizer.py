import math
from collections import deque
import julia


class JuliaCar:
    def __init__(self, agent):
        parameters = agent.control_params['optimizer_params']
        self.dt = parameters['plan_time_precision']
        self.T = parameters['plan_time_horizon']
        self.ipx = agent.state.tpx
        self.x = agent.state.x
        self.y = agent.state.y
        self.v = agent.state.v
        self.da = agent.state.dv
        self.theta = agent.state.heading

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
        self.track = agent.track
        self.actions = []

    def race_optimize(self, opponent_cars=None):
        if opponent_cars:
            opponent_cars = list(filter(lambda c: c != self, opponent_cars))
        else:
            opponent_cars = []


def julia_race_optimize(agent, opponent_cars, replan_time, input_update_time):
    agent_b_car = JuliaCar(agent)
    opp_b_cars = list(map(lambda c: JuliaCar(c), opponent_cars))
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
    actions = deque(agent_b_car.actions)
    return actions
