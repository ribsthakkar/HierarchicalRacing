import itertools
import math

import numpy as np
from shapely import geometry as geom

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