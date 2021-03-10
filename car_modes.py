from enum import Enum

class DriveModes(Enum):
    FOLLOW = 1
    PASS = 2
    RACE = 3
    BLOCK = 4


class InputModes:
    def __init__(self, max_acceleration, max_braking, max_cornering_gs, max_velocity, max_steering_angle):
        pass

    def mode_from_velocity_heading(self, velocity, heading):
        pass

    def try_switch(self, mode, velocity, heading, slip, time_step):
        pass
