import math
from functools import lru_cache, reduce

from util import dist


@lru_cache(maxsize=None)
def comb(n,r):
    f = math.factorial
    return f(n) // f(r) // f(n-r)


def bezier_trajectory(c, tau, bezier_order, time_horizon):
    return reduce(lambda x,y: x+y,
                  map(lambda i: c[i] * comb(bezier_order, i) * ((1-tau/time_horizon)**(bezier_order-i)) * (tau/time_horizon)**i,
                      range(0, bezier_order+1)))


def bezier_speed(c, tau, bezier_order, time_horizon):
    return bezier_order/time_horizon * \
           reduce(lambda x, y: x + y,
                  map(lambda i: (c[i+1] - c[i]) * comb(bezier_order - 1, i) * ((1 - tau/time_horizon) ** (bezier_order - 1 - i)) * (tau/time_horizon) ** i,
                      range(0, bezier_order)))


def bezier_acceleration(c, tau, bezier_order, time_horizon):
    return (bezier_order*(bezier_order-1)) / time_horizon**2 * \
           reduce(lambda x, y: x + y,
                  map(lambda i: (c[i+2] - 2*c[i+1] + c[i]) * comb(bezier_order - 2, i) * ((1 - tau/time_horizon) ** (bezier_order - 2 - i)) * (tau/time_horizon) ** i,
                      range(0, bezier_order-1)))


def bezier_arc_length(cx, cy, time_horizon, precision=0.05,):
    tau1 = 0
    tau2 = tau1 + precision
    d = 0.0
    bezier_order = len(cx) - 1
    while tau1 < time_horizon:
        update = dist(bezier_trajectory(cx, tau2, bezier_order, time_horizon), bezier_trajectory(cy, tau2, bezier_order, time_horizon), bezier_trajectory(cx, tau1, bezier_order, time_horizon), bezier_trajectory(cy, tau1, bezier_order, time_horizon))
        d += update
        tau1 = tau2
        tau2 += precision
    return d