from scipy.optimize import minimize, NonlinearConstraint, LinearConstraint, Bounds
import math
import numpy as np

coefficient_of_friction = 1.5
gravitational_acceleration = 9.8

# Straight line beginning to curve
# init_dx_dt = 44
# init_dy_dt = 0
# init_x = -75
# init_y = 42
# target_x = 0
# target_y = 48
# c_x = [init_x, (init_x+init_dx_dt/4), init_x, target_x, target_x]
# c_y = [init_y, (init_y+init_dy_dt/4), init_x, target_x, target_y]
# c = c_x + c_y
# initial = np.array(c)
# max_accel = 15
# min_accel = -20
# max_vel = 27
# track_width = 10
# car_width = 1.6
# track_points_x = [-75, -71.25, -67.5, -63.75, -60.0, -56.25, -52.5, -48.75, -45.0, -41.25, -37.5, -33.75, -30.0, -26.25, -22.5, -18.75, -15.0, -11.25, -7.5, -3.75, 0.0]
# track_points_y = [42, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0]


# max_accel = 15
# min_accel = -20
# max_vel = 27
# track_width = 10
# car_width = 1.6
# track_points_x = [-20, -18.0, -16.0, -14.0, -12.0, -10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0]
# track_points_y = [45.00000000135352, 45.00000001000122, 45.0000000738996, 45.000000546048234, 45.000004034779266, 45.000029813114274, 45.00022028552892, 45.001627415824444, 45.012009458053896, 45.087895541044745, 45.60686544016995, 48.020325350385164, 51.540595809549146, 52.76550215398915, 52.96743889827283, 52.99557777090461, 52.99940123017991, 52.99991896007215, 52.99998903234234, 52.99999851568718]
# init_dx_dt = 27
# init_dy_dt = 0
# init_d2x_dt = -10
# init_d2y_dt = 0
# init_x = -18
# init_y = 42
# target_x = track_points_x[-1]
# target_y = track_points_y[-1]
# c_x = [init_x, (init_x+init_dx_dt/4), init_d2x_dt/12 + 2 * ((init_x+init_dx_dt/4)) - init_x, (init_x+target_x)/2, target_x]
# c_y = [init_y, (init_y+init_dy_dt/4), init_d2y_dt/12 + 2 * ((init_y+init_dy_dt/4)) - init_y, (init_y+target_y)/2, target_y]
# c = c_x + c_y
# initial = np.array(c)

max_accel = 15
min_accel = -20
max_vel = 27
track_width = 10
car_width = 1.6
track_points_x = [-5, -3.75, -2.5, -1.25, 0.0, 1.25, 2.5, 3.75, 5.0, 6.25, 7.5, 8.75, 10.0, 11.25, 12.5, 13.75, 15.0, 16.25, 17.5, 18.75]
track_points_y = [45.00442222909539, 45.01541387730662, 45.05354280739428, 45.1838189592802, 45.60686544016995, 46.78160111060247, 49.0, 51.21839888939753, 52.39313455983005, 52.8161810407198, 52.94645719260572, 52.98458612269338, 52.99557777090461, 52.99873251024718, 52.99963681705038, 52.999895942972266, 52.999970186885726, 52.99999145837704, 52.99999755278218, 52.9999992988602]
init_dx_dt = 8
init_dy_dt = 10
init_d2x_dt = 5
init_d2y_dt = 5
init_x = -5
init_y = 46
target_x = track_points_x[-1]
target_y = track_points_y[-1]
c_x = [init_x, (init_x+init_dx_dt/4), init_d2x_dt/12 + 2 * ((init_x+init_dx_dt/4)) - init_x, (init_x+target_x)/2, target_x]
c_y = [init_y, (init_y+init_dy_dt/4), init_d2y_dt/12 + 2 * ((init_y+init_dy_dt/4)) - init_y, (init_y+target_y)/2, target_y]
c = c_x + c_y
initial = np.array(c)

print(c_x)
print(c_y)


def trajectory_x(cx, tau):
    return (cx[0] * (1 - tau) ** 4) + (4 * cx[1] * tau * ((1 - tau) ** 3)) + (6 * cx[2] * (tau ** 2) * ((1 - tau) ** 2)) \
           + (4 * cx[3] * (tau ** 3) * (1 - tau)) + cx[4] * tau ** 4
    pass


def trajectory_y(cy, tau):
    return cy[0] * (1 - tau) ** 4 + cy[1] * 4 * tau * (1 - tau) ** 3 + cy[2] * 6 * (tau ** 2) * ((1 - tau) ** 2) \
           + cy[3] * 4 * (tau ** 3) * ((1 - tau)) + cy[4] * tau ** 4
    pass


def speed_x(cx, tau):
    return 4 * ((cx[1] - cx[0]) * (1 - tau) ** 3 + 3 * (cx[2] - cx[1]) * (1 - tau) ** 2 * tau
                + 3 * (cx[3] - cx[2]) * (1 - tau) * tau ** 2 + (cx[4] - cx[3]) * (tau) ** 3)
    pass


def speed_y(cy, tau):
    return 4 * ((cy[1] - cy[0]) * (1 - tau) ** 3 + (cy[2] - cy[1]) * (1 - tau) ** 2 * tau
                + (cy[3] - cy[2]) * (1 - tau) * tau ** 2 + (cy[4] - cy[3]) * (tau) ** 3)
    pass


def acceleration_x(cx, tau):
    return 12 * ((cx[2] - 2 * cx[1] + cx[0]) * (1 - tau) ** 2 + 2 * (cx[3] - 2 * cx[2] + cx[2]) * tau * (1 - tau)
                 + (cx[4] - 2 * cx[3] + cx[2]) * tau ** 2)
    pass


def acceleration_y(cy, tau):
    return 12 * ((cy[2] - 2 * cy[1] + cy[0]) * (1 - tau) ** 2 + 2 * (cy[3] - 2 * cy[2] + cy[2]) * tau * (1 - tau)
                 + (cy[4] - 2 * cy[3] + cy[2]) * tau ** 2)
    pass


def curvature(cx, cy, tau):
    x_p = speed_x(cx, tau)
    x_pp = acceleration_x(cx, tau)
    y_p = speed_y(cy, tau)
    y_pp = acceleration_y(cy, tau)
    # print(x_p, x_pp, y_p, y_pp)
    if x_p == 0 and y_p == 0:
        return np.inf
    return abs(x_p*y_pp - y_p*x_pp)/(x_p**2 + y_p**2)**(3/2)


def radius(cx, cy, tau):
    c = curvature(cx, cy, tau)
    # print(c)
    return 9999999 if c == 0 else 1/c


def dist(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)


def distance_to_center(x, y):
    min_dist = np.inf
    for i in range(len(track_points_x)):
        min_dist = min(dist(x, y, track_points_x[i],track_points_y[i]), min_dist)
    return min_dist
    pass


def arc_length(cx, cy, delta=0.05):
    tau1 = 0
    tau2 = tau1 + delta
    d = 0.0
    while tau1 < 1:
        d += dist(trajectory_x(cx, tau2), trajectory_y(cy, tau2), trajectory_x(cx, tau1), trajectory_y(cy, tau1))
        # print(tau2, trajectory_x(c_x, tau2), trajectory_y(c_y, tau2))
        # print(tau1, trajectory_x(cx, tau1), trajectory_y(cy, tau1))
        tau1 = tau2
        tau2 += delta
    return d

# print(arc_length(initial[:5], initial[5:]))
# # exit(0)
# print(speed_x(initial[:5], 0), speed_y(initial[5:], 0))
# print(math.sqrt(coefficient_of_friction * gravitational_acceleration * radius(c[:5], c[5:], 0)))
# exit(1)
# opt = lambda c: - arc_length(c[:5], c[5:]) - math.sqrt(speed_x(c[:5], 1)**2 + speed_y(c[5:], 1)**2)
opt = lambda c: - math.sqrt(speed_x(c[:5], 1)**2 + speed_y(c[5:], 1)**2)

constraints = []

# Speed Constraints
t = 0
while t <= 1:
    con = lambda c:  math.sqrt(speed_x(c[:5], t)**2 + speed_y(c[5:], t)**2) - math.sqrt(coefficient_of_friction * gravitational_acceleration * radius(c[:5], c[5:], t))
    nlc = NonlinearConstraint(con, -100, 0)
    # print(t, con(initial))
    constraints.append(nlc)
    t += 0.05

con = lambda c: speed_x(c[:5], 0) - init_dx_dt
nlc = NonlinearConstraint(con, 0, 0)
constraints.append(nlc)
#
con = lambda c: speed_y(c[5:], 0) - init_dy_dt
nlc = NonlinearConstraint(con, 0, 0)
constraints.append(nlc)

# Acceleration Constraints
t = 0
while t <= 1:
    con = lambda c:  math.sqrt(speed_x(c[:5], t)**2 + speed_y(c[5:], t)**2)
    nlc = NonlinearConstraint(con, min_accel, max_accel)
    constraints.append(nlc)
    # print(t, con(initial))
    t += 0.05

# Control Points Constraints
# t = 0
# A = [[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 1, 0, 0, 0, 0]]
# lc = LinearConstraint(np.array(A), [init_x, init_y], [init_x, init_y])
# constraints.append(lc)

# Track Bounds Constraints
t = 0
while t <= 1:
    con = lambda c: distance_to_center(trajectory_x(c[:5], t), trajectory_y(c[5:], t))
    nlc = NonlinearConstraint(con, 0, (track_width/2)-car_width)
    constraints.append(nlc)
    # print(t, con(initial))
    t += 0.05
# exit(0)
print(initial)
bounds = Bounds([init_x, (init_x+init_dx_dt/4), init_d2x_dt/12 + 2 * ((init_x+init_dx_dt/4)) - init_x, init_x, target_x-2,
                 init_y, (init_y+init_dy_dt/4), init_d2y_dt/12 + 2 * ((init_y+init_dy_dt/4)) - init_y, init_y, target_y-2],

                [init_x, (init_x+init_dx_dt/4), init_d2x_dt/12 + 2 * ((init_x+init_dx_dt/4)) - init_x, target_x, target_x+2,
                 init_y, (init_y+init_dy_dt/4), init_d2y_dt/12 + 2 * ((init_y+init_dy_dt/4)) - init_y, target_y, target_y+2])
# bounds = Bounds([init_x, -50, -50, -50, target_x, init_y, -50, -50, -50, target_y],
#                 [init_x, 50, 50, 50, target_x, init_y, 50, 50, 50, target_y])
result = minimize(opt, initial, bounds=bounds, constraints=constraints, options={'disp': True})

print(result.x)

output = [round(x) for x in result.x]
print(output[:5])
print(output[5:])
print(arc_length(output[:5], output[5:]))
print(speed_x(output[:5],1), speed_y(output[5:],1))