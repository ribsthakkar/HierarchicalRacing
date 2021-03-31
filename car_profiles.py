from util import gravitational_acceleration


def f1_acc_profile(sp):
    if 0 <= sp < 100:
        return 10
    else:
        return 1

f1_profile = {
    'max_velocity': 100,
    'max_acceleration': 10,
    'acceleration_profile': f1_acc_profile,
    'car_width': 2,
    'car_length': 5,
    'max_cornering_gs': 5,
    'max_steering_angle': 40,
    'max_braking': 3*gravitational_acceleration
}


def mclaren720s_acc_profile(sp):
    if 0 <= sp < 90:
        return 10
    else:
        return 1

mclaren720s_profile = {
    'max_velocity': 90,
    'max_acceleration': 10,
    'acceleration_profile': mclaren720s_acc_profile,
    'car_width': 1.9,
    'car_length': 4.5,
    'max_cornering_gs': 1,
    'max_steering_angle': 40,
    'max_braking': 2*gravitational_acceleration
}

def basicsports_acc_profile(sp):
    if 0 <= sp < 40:
        return 5
    else:
        return 1

basicsports_profile = {
    'max_velocity': 40,
    'max_acceleration': 5,
    'acceleration_profile': basicsports_acc_profile,
    'car_width': 1.8,
    'car_length': 4,
    'max_cornering_gs': .8,
    'max_steering_angle': 40,
    'max_braking': 2*gravitational_acceleration
}

def example_acc_profile(sp):
    if 0 <= sp < 3:
        return 1
    else:
        return 1

example_profile = {
    'max_velocity': 3,
    'max_acceleration': 1,
    'acceleration_profile': example_acc_profile,
    'car_width': .18,
    'car_length': .4,
    'max_cornering_gs': 1,
    'max_steering_angle': 40,
    'max_braking': .5*gravitational_acceleration
}