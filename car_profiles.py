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


def sports_acc_profile(sp):
    if 0 <= sp < 90:
        return 10
    else:
        return 1

sports_profile = {
    'max_velocity': 90,
    'max_acceleration': 10,
    'acceleration_profile': sports_acc_profile,
    'car_width': 4,
    'car_length': 6,
    'max_cornering_gs': 1,
    'max_steering_angle': 40,
    'max_braking': 2*gravitational_acceleration
}