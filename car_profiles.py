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
}
