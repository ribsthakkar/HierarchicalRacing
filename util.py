import itertools
import math

gravitational_acceleration = 9.8

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