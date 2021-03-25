# import sys
# from io import StringIO
#
# oldstdin = sys.stdin
# sys.stdin = StringIO('Test')
#
# print(input('testing,'))
import math
from scipy import optimize
import numpy as np
from car_modes import TPI
from util import round_to_fraction

initialv = 8
initalh = math.pi/2

targetv = 8
targeth = 0.95 * math.pi/2

dt = .5

accx = ((targetv-initialv)/dt) * (math.cos(initalh)) + initialv*((targeth-initalh)/dt)*(-math.sin(initalh))
accy = ((targetv-initialv)/dt) * (math.sin(initalh)) + initialv*((targeth-initalh)/dt)*(math.cos(initalh))
print(accx, accy)

print(np.unwrap(np.array([24.5,])))
# def mode_from_velocity_heading(velocity, heading):
#     v = round_to_fraction(velocity, 0.5)
#     v = max(min(v, 100), 0.5)
#     h = round_to_fraction(heading, TPI/100)
#     # h = max(min(h, TPI), 0)
#     return (v, h)
# print(mode_from_velocity_heading(54.989175050172946, 10.638153599774203))

print(round_to_fraction(5.930465589273715, TPI/100))

# vxi = initialv*math.cos(initalh)
# vyi = initialv*math.sin(initalh)
# vxf = targetv*math.cos(targeth)
# vyf = targetv*math.sin(targeth)
# ax = (vxf-vxi)/dt
# ay = (vyf-vyi)/dt
# a = math.sqrt((ax**2) + (ay**2))
#
# # a_t = (vxi*ax + vyi*ay)/initialv
# # a_n = math.sqrt((a**2) - (a_t)**2)
# # print(a, a_t, a_n)
# #
# #
# # a_t = (vxf*ax + vyf*ay)/targetv
# # a_n = math.sqrt((a**2) - (a_t)**2)
# # print(a, a_t, a_n)
#
#
# def opt_t(t):
#     a_t = 0.5 * (2*ax*(vxi + ax * t) + 2*ay*(vyi + ay * t))/(math.sqrt((vxi + ax * t)**2 + (vyi + ay * t)**2))
#     return -(a_t**2)
#
# def opt_n(t):
#     a_t = 0.5 * (2*ax*(vxi + ax * t) + 2*ay*(vyi + ay * t))/(math.sqrt((vxi + ax * t)**2 + (vyi + ay * t)**2))
#     a_n = math.sqrt((a**2) - (a_t)**2)
#     return -(a_n**2)
#
# result = optimize.minimize_scalar(opt_t, bounds=(0, dt), method='bounded')
# print(math.sqrt(-result.fun), result.x)
#
# result = optimize.minimize_scalar(opt_n, bounds=(0, dt), method='bounded')
# print(math.sqrt(-result.fun), result.x)
#
# import scipy.stats as stats
#
# lower, upper = 3.5, 6
# mu, sigma = 5, 0.7
# X = stats.truncnorm(
#     (lower - mu) / sigma, (upper - mu) / sigma, loc=mu, scale=sigma)
#
# print(type(X.rvs(1)[0]))
#
# print(round_to_fraction(1.5, 0.5))
# print(max(min(1.3, 100), .5))
