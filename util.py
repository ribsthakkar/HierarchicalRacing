from shapely import geometry as geom
import itertools
import math

import numpy as np

gravitational_acceleration = 9.8
TPI = 2 * math.pi

def circ_slice(a, start, length):
    #from StackOverflow
    it = itertools.cycle(a)
    next(itertools.islice(it, start, start), None)
    return list(itertools.islice(it, length))

def dist(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def round_to_fraction(num, frac):
    a = num * (1/frac)
    a = round(a)
    return a * (frac)

def is_cw(current, target):
    return math.sin(current - target) > 0

def find_smallest_rotation(target_h, current_h):
    if math.isclose(target_h, current_h):
        return 0
    if is_cw(current_h, target_h):
        rotation = target_h-current_h if current_h > target_h else -(current_h + TPI - target_h)
        return rotation
    else:
        rotation = target_h - current_h if target_h > current_h else target_h + TPI - current_h
        return rotation


def rect_from_center(x, y, l, w, rot):
    # Adapted from stackoverflow here: https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
    tr = np.array([x + ((l / 2) * math.cos(rot)) - ((w / 2) * math.sin(rot)), y + ((l / 2) * math.sin(rot)) + ((w / 2) * math.cos(rot))])
    tl = np.array([x - ((l / 2) * math.cos(rot)) - ((w / 2) * math.sin(rot)), y - ((l / 2) * math.sin(rot)) + ((w / 2) * math.cos(rot))])
    bl = np.array([x - ((l / 2) * math.cos(rot)) + ((w / 2) * math.sin(rot)), y - ((l / 2) * math.sin(rot)) - ((w / 2) * math.cos(rot))])
    br = np.array([x + ((l / 2) * math.cos(rot)) + ((w / 2) * math.sin(rot)), y + ((l / 2) * math.sin(rot)) - ((w / 2) * math.cos(rot))])
    points = np.vstack([tr, tl, bl, br])
    return geom.Polygon(points)


def generate_arc_points(centerx, centery, radius, start_angle, end_angle, num_segments=100):
    # Code snippet adapted from: https://stackoverflow.com/questions/30762329/how-to-create-polygons-with-arcs-in-shapely-or-a-better-library
    # The coordinates of the arc
    theta = np.linspace(start_angle, end_angle, num_segments)
    x = centerx + radius * np.cos(theta)
    y = centery + radius * np.sin(theta)
    return np.column_stack([x, y])


def generate_heading_sweep(car, time_step, from_center=math.pi/4):
    centerx, centery = car.state.x + (car.state.l / 2) * math.cos(car.state.heading), car.state.y + (car.state.l / 2) * math.sin(car.state.heading)
    tr = np.array([car.state.x + ((car.state.l / 2) * math.cos(car.state.heading)) - ((car.state.w / 2) * math.sin(car.state.heading)),
                   car.state.y + ((car.state.l / 2) * math.sin(car.state.heading)) + ((car.state.w / 2) * math.cos(car.state.heading))])
    br = np.array([car.state.x + ((car.state.l / 2) * math.cos(car.state.heading)) + ((car.state.w / 2) * math.sin(car.state.heading)),
                   car.state.y + ((car.state.l / 2) * math.sin(car.state.heading)) - ((car.state.w / 2) * math.cos(car.state.heading))])
    radius = car.state.v * time_step
    start_angle, end_angle = car.state.heading-from_center, car.state.heading+from_center  # In degrees
    arc_points = generate_arc_points(centerx, centery, radius, start_angle, end_angle)
    poly_points = np.vstack([arc_points, tr, br])
    sweep = geom.Polygon(poly_points)
    return sweep


def pos_estimate_functions(xi, yi, vi, vf, hi, hf, dt):
    vxi = vi * math.cos(hi)
    vyi = vi * math.sin(hi)
    vxf = vf * math.cos(hf)
    vyf = vf * math.sin(hf)
    ax = (vxf - vxi) / dt
    ay = (vyf - vyi) / dt
    x_pos = lambda t: xi +vxi*t+0.5*ax*t**2
    y_pos = lambda t: yi + vyi*t+0.5*ay*t**2
    h_pos = lambda t: math.atan2((vyi + ay*t),(vxi + ax*t))
    return x_pos,y_pos, h_pos

def is_greater_than(a, b, rel_tol = 1e-9, abs_tol=1e-6):
    if math.isclose(a, b, rel_tol=rel_tol, abs_tol=abs_tol):
        return False
    else:
        return a > b


def log_barrier_function_value(x, b):
    if x > b: return (x-b)*999
    return -math.log(b-x)

def log_leq_barrier_function_value(x, b, rel_tol=1e-5, base=math.e):
    if x > (b+1)*(1+rel_tol): return (x-(b+1)*rel_tol)*-math.log(1e-9, base)
    return -math.log((b+1)*(1+rel_tol)-x, base)