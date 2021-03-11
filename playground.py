# import sys
# from io import StringIO
#
# oldstdin = sys.stdin
# sys.stdin = StringIO('Test')
#
# print(input('testing,'))

from bezier_optimizer import BezierCar

b = BezierCar.compute_trajectory_intersection()