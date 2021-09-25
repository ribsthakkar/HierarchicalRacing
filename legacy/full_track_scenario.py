from collections import deque

from scipy.optimize import minimize, NonlinearConstraint, LinearConstraint, Bounds, basinhopping
from scipy.interpolate import interp1d, splev, splrep, BSpline, splprep, SmoothBivariateSpline
import math
import numpy as np
import matplotlib.pyplot as plt
import shapely.geometry as geom

coefficient_of_friction = 1.5
gravitational_acceleration = 9.8
max_accel = 15
min_accel = -20
max_vel = 94
track_width = 10
car_width = 1.6
time_delta = 0.05
main_track_x = (67, 65, 64, 64, 63, 62, 61, 61, 60, 60, 60, 59, 59, 59, 59, 59, 59, 59, 59, 59, 60, 60, 60, 61, 61, 62, 62, 63, 64, 64, 65, 66, 66, 67, 68, 69, 69, 70, 71, 72, 73, 74, 74, 75, 76, 77, 78, 79, 79, 80, 81, 82, 82, 83, 84, 84, 85, 86, 86, 87, 87, 88, 88, 88, 89, 89, 89, 90, 90, 90, 90, 90, 90, 90, 91, 91, 91, 91, 91, 91, 91, 90, 90, 90, 90, 90, 90, 90, 90, 90, 89, 89, 89, 89, 89, 89, 88, 88, 88, 88, 88, 88, 88, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 88, 88, 88, 88, 89, 89, 89, 90, 90, 91, 91, 92, 92, 93, 93, 94, 95, 96, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 107, 108, 109, 110, 112, 113, 114, 116, 117, 119, 120, 122, 123, 125, 127, 128, 130, 132, 133, 135, 137, 139, 141, 143, 144, 146, 148, 150, 152, 154, 156, 158, 160, 163, 165, 167, 169, 171, 173, 176, 178, 180, 182, 185, 187, 189, 192, 194, 196, 199, 201, 204, 206, 208, 211, 213, 216, 218, 221, 223, 226, 228, 231, 233, 236, 239, 241, 244, 246, 249, 251, 254, 257, 259, 262, 265, 267, 270, 272, 275, 278, 280, 283, 286, 288, 291, 293, 296, 299, 301, 304, 307, 309, 312, 314, 317, 320, 322, 325, 327, 330, 332, 335, 338, 340, 343, 345, 348, 350, 353, 355, 358, 360, 363, 365, 367, 370, 372, 375, 377, 379, 382, 384, 386, 389, 391, 393, 396, 398, 400, 402, 405, 407, 409, 411, 413, 415, 418, 420, 422, 424, 426, 428, 430, 432, 434, 436, 438, 439, 441, 443, 445, 447, 449, 450, 452, 454, 456, 457, 459, 460, 462, 464, 465, 467, 468, 470, 471, 472, 474, 475, 476, 478, 479, 480, 481, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 494, 495, 496, 497, 498, 498, 499, 500, 501, 501, 502, 503, 503, 504, 504, 505, 506, 506, 507, 507, 508, 508, 509, 509, 510, 510, 511, 511, 512, 512, 513, 513, 514, 514, 514, 515, 515, 516, 516, 517, 517, 518, 518, 518, 519, 519, 520, 520, 521, 521, 522, 522, 523, 523, 524, 524, 525, 526, 526, 527, 527, 528, 529, 529, 530, 531, 531, 532, 533, 534, 534, 535, 536, 537, 538, 539, 539, 540, 541, 542, 543, 544, 545, 547, 548, 549, 550, 551, 552, 554, 555, 556, 557, 559, 560, 561, 563, 564, 566, 567, 568, 570, 571, 573, 574, 576, 577, 579, 580, 582, 584, 585, 587, 588, 590, 592, 593, 595, 597, 598, 600, 602, 604, 605, 607, 609, 611, 612, 614, 616, 618, 619, 621, 623, 625, 627, 628, 630, 632, 634, 636, 638, 639, 641, 643, 645, 647, 649, 650, 652, 654, 656, 658, 659, 661, 663, 665, 667, 668, 670, 672, 674, 676, 677, 679, 681, 683, 684, 686, 688, 689, 691, 693, 695, 696, 698, 699, 701, 703, 704, 706, 707, 709, 711, 712, 714, 715, 717, 718, 720, 721, 722, 724, 725, 727, 728, 729, 731, 732, 733, 734, 736, 737, 738, 739, 740, 741, 742, 743, 744, 746, 746, 747, 748, 749, 750, 751, 752, 753, 753, 754, 755, 755, 756, 756, 757, 757, 758, 758, 758, 759, 759, 759, 759, 759, 759, 759, 759, 759, 759, 758, 758, 758, 757, 757, 756, 755, 755, 754, 753, 752, 751, 750, 748, 747, 746, 744, 743, 741, 740, 738, 736, 734, 733, 731, 729, 727, 725, 723, 721, 718, 716, 714, 712, 710, 707, 705, 703, 700, 698, 696, 693, 691, 689, 686, 684, 682, 679, 677, 675, 673, 670, 668, 666, 664, 662, 660, 657, 655, 653, 651, 649, 647, 645, 643, 641, 640, 638, 636, 634, 632, 630, 628, 627, 625, 623, 621, 620, 618, 616, 614, 613, 611, 609, 608, 606, 604, 603, 601, 599, 598, 596, 595, 593, 591, 590, 588, 587, 585, 583, 582, 580, 578, 577, 575, 574, 572, 570, 569, 567, 565, 564, 562, 560, 559, 557, 555, 554, 552, 550, 549, 547, 545, 544, 542, 540, 538, 537, 535, 533, 531, 530, 528, 526, 524, 523, 521, 519, 517, 515, 514, 512, 510, 508, 506, 504, 503, 501, 499, 497, 495, 493, 491, 490, 488, 486, 484, 482, 480, 478, 476, 474, 472, 470, 468, 466, 465, 463, 461, 459, 457, 455, 453, 451, 449, 447, 445, 443, 441, 439, 436, 434, 432, 430, 428, 426, 424, 422, 420, 418, 416, 414, 412, 409, 407, 405, 403, 401, 399, 397, 394, 392, 390, 388, 386, 384, 381, 379, 377, 375, 372, 370, 368, 366, 364, 361, 359, 357, 354, 352, 350, 348, 345, 343, 341, 338, 336, 334, 331, 329, 327, 325, 322, 320, 318, 315, 313, 311, 308, 306, 304, 302, 299, 297, 295, 293, 290, 288, 286, 284, 282, 280, 277, 275, 273, 271, 269, 267, 265, 263, 261, 259, 257, 255, 253, 251, 249, 247, 246, 244, 242, 240, 239, 237, 235, 234, 232, 231, 229, 228, 226, 225, 223, 222, 221, 219, 218, 217, 216, 214, 213, 212, 211, 210, 209, 208, 207, 207, 206, 205, 204, 204, 203, 202, 202, 201, 200, 200, 199, 199, 198, 198, 197, 197, 196, 196, 196, 195, 195, 195, 194, 194, 194, 193, 193, 193, 192, 192, 192, 192, 191, 191, 191, 190, 190, 190, 190, 189, 189, 189, 188, 188, 188, 187, 187, 186, 186, 186, 185, 185, 184, 184, 183, 183, 182, 181, 181, 180, 179, 179, 178, 177, 176, 175, 174, 173, 172, 171, 170, 169, 168, 167, 166, 164, 163, 162, 160, 159, 157, 156, 154, 152, 151, 149, 147, 145, 143, 141, 139, 137, 135, 133, 131, 128, 126, 124, 121, 119, 117, 115, 112, 110, 108, 105, 103, 101, 99, 96, 94, 92, 90, 88, 86, 84, 82, 80, 78, 77, 75, 73, 72, 70, 69, 68, 66)
main_track_y = (343, 343, 342, 341, 340, 339, 338, 336, 335, 334, 332, 331, 330, 328, 326, 325, 323, 322, 320, 318, 316, 314, 312, 310, 309, 307, 304, 302, 300, 298, 296, 294, 291, 289, 287, 285, 282, 280, 278, 275, 273, 270, 268, 265, 263, 260, 258, 255, 253, 250, 248, 245, 243, 240, 237, 235, 232, 230, 227, 224, 222, 219, 217, 214, 212, 209, 206, 204, 201, 199, 196, 194, 191, 189, 186, 184, 181, 179, 176, 174, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 146, 144, 141, 139, 137, 135, 133, 131, 129, 126, 124, 122, 120, 118, 117, 115, 113, 111, 109, 107, 106, 104, 102, 100, 99, 97, 96, 94, 92, 91, 90, 88, 87, 85, 84, 83, 82, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 71, 70, 69, 69, 68, 67, 67, 66, 66, 65, 65, 64, 64, 63, 63, 63, 63, 62, 62, 62, 62, 62, 61, 61, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62, 62, 62, 62, 63, 63, 63, 63, 64, 64, 64, 65, 65, 65, 66, 66, 66, 67, 67, 67, 68, 68, 69, 69, 70, 70, 70, 71, 71, 72, 72, 73, 73, 74, 74, 74, 75, 75, 76, 76, 77, 77, 78, 78, 78, 79, 79, 80, 80, 80, 81, 81, 81, 82, 82, 82, 83, 83, 83, 84, 84, 84, 84, 85, 85, 85, 85, 85, 85, 85, 85, 85, 86, 86, 86, 85, 85, 85, 85, 85, 85, 85, 85, 85, 84, 84, 84, 84, 84, 83, 83, 83, 83, 82, 82, 82, 82, 82, 81, 81, 81, 81, 80, 80, 80, 80, 79, 79, 79, 79, 79, 78, 78, 78, 78, 78, 78, 78, 78, 77, 77, 77, 77, 77, 78, 78, 78, 78, 78, 78, 78, 79, 79, 79, 79, 80, 80, 81, 81, 82, 82, 83, 83, 84, 84, 85, 86, 87, 88, 88, 89, 90, 91, 92, 94, 95, 96, 97, 98, 100, 101, 102, 104, 105, 107, 108, 110, 112, 113, 115, 116, 118, 120, 122, 124, 125, 127, 129, 131, 133, 135, 137, 139, 141, 143, 145, 147, 149, 151, 153, 155, 158, 160, 162, 164, 166, 169, 171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193, 195, 198, 200, 202, 205, 207, 209, 211, 214, 216, 218, 220, 222, 225, 227, 229, 231, 233, 235, 238, 240, 242, 244, 246, 248, 250, 252, 254, 256, 258, 260, 262, 263, 265, 267, 269, 271, 272, 274, 276, 277, 279, 281, 282, 284, 285, 287, 288, 290, 291, 293, 294, 296, 297, 298, 300, 301, 302, 304, 305, 306, 308, 309, 310, 311, 312, 314, 315, 316, 317, 318, 319, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 365, 366, 367, 368, 369, 370, 371, 372, 374, 375, 376, 377, 378, 380, 381, 382, 384, 385, 386, 388, 389, 390, 392, 393, 394, 396, 397, 399, 400, 402, 403, 405, 407, 408, 410, 411, 413, 415, 417, 418, 420, 422, 424, 426, 427, 429, 431, 433, 435, 437, 439, 442, 444, 446, 448, 450, 452, 455, 457, 459, 461, 464, 466, 468, 471, 473, 475, 478, 480, 482, 484, 487, 489, 491, 493, 495, 497, 500, 502, 504, 506, 508, 510, 511, 513, 515, 517, 519, 520, 522, 523, 525, 526, 527, 529, 530, 531, 532, 533, 534, 535, 535, 536, 537, 537, 538, 538, 539, 539, 539, 539, 540, 540, 540, 540, 540, 540, 540, 540, 539, 539, 539, 538, 538, 538, 537, 537, 536, 535, 535, 534, 534, 533, 532, 531, 530, 530, 529, 528, 527, 526, 525, 524, 523, 522, 521, 520, 519, 518, 517, 515, 514, 513, 512, 511, 510, 509, 507, 506, 505, 504, 503, 502, 500, 499, 498, 497, 496, 495, 494, 492, 491, 490, 489, 488, 487, 486, 485, 484, 483, 482, 482, 481, 480, 479, 478, 478, 477, 476, 476, 475, 474, 474, 473, 473, 473, 472, 472, 472, 471, 471, 471, 471, 470, 470, 470, 470, 470, 470, 470, 470, 470, 470, 471, 471, 471, 471, 471, 472, 472, 472, 473, 473, 473, 474, 474, 475, 475, 476, 476, 477, 477, 478, 479, 479, 480, 480, 481, 482, 483, 483, 484, 485, 486, 486, 487, 488, 489, 490, 491, 491, 492, 493, 494, 495, 496, 497, 498, 499, 500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519, 520, 522, 523, 524, 525, 526, 527, 528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539, 540, 541, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551, 551, 552, 553, 554, 555, 555, 556, 557, 558, 558, 559, 560, 560, 561, 561, 562, 562, 563, 563, 564, 564, 564, 564, 565, 565, 565, 565, 565, 565, 565, 565, 565, 565, 565, 565, 564, 564, 564, 563, 563, 562, 562, 561, 560, 560, 559, 558, 557, 556, 555, 554, 553, 552, 550, 549, 547, 546, 544, 543, 541, 539, 538, 536, 534, 532, 530, 528, 526, 524, 522, 520, 518, 516, 514, 512, 510, 508, 506, 503, 501, 499, 497, 494, 492, 490, 488, 485, 483, 481, 478, 476, 474, 471, 469, 467, 464, 462, 460, 457, 455, 453, 450, 448, 446, 443, 441, 439, 436, 434, 432, 430, 427, 425, 423, 421, 419, 417, 414, 412, 410, 408, 406, 404, 402, 400, 398, 396, 394, 393, 391, 389, 387, 386, 384, 382, 381, 379, 377, 376, 374, 373, 372, 370, 369, 368, 367, 365, 364, 363, 362, 361, 360, 360, 359, 358, 357, 357, 356, 356, 355, 355, 354, 354, 354, 353, 353, 353, 353, 352, 352, 352, 352, 352, 351, 351, 351, 351, 351, 351, 351, 350, 350, 350, 350, 349, 349, 349, 348, 348, 347, 347, 346, 346, 345, 344, 344)

point_horizon = 30
position_index = 60
track_points_x = list(main_track_x[position_index:position_index+point_horizon])
track_points_y = list(main_track_y[position_index:position_index+point_horizon])


# class NearestPoint(object):
#     def __init__(self, line, ax):
#         self.line = line
#         self.ax = ax
#         ax.figure.canvas.mpl_connect('button_press_event', self)
#
#     def __call__(self, event):
#         x, y = event.xdata, event.ydata
#         point = geom.Point(x, y)
#         distance = self.line.distance(point)
#         self.draw_segment(point)
#         print('Distance to line:', distance)
#
#     def draw_segment(self, point):
#         point_on_line = line.interpolate(line.project(point))
#         self.ax.plot([point.x, point_on_line.x], [point.y, point_on_line.y],
#                      color='red', marker='o', scalex=False, scaley=False)
#         fig.canvas.draw()
#
# fig, ax = plt.subplots()
# ax.plot(*track_horizon.T)
# ax.axis('equal')
# NearestPoint(line, ax)
# plt.show()


init_dx_dt = -2
init_dy_dt = -20
init_d2x_dt = -5
init_d2y_dt = -5
init_x = main_track_x[position_index]
init_y = main_track_y[position_index]
target_x = track_points_x[-1]
target_y = track_points_y[-1]


# translate_x = init_x
# translate_y  = init_y
# for i in range(point_horizon):
#     track_points_x[i] -= init_x
#     track_points_y[i] -= init_y
#
# init_x = 0
# init_y = 0
# target_x -= translate_x
# target_y -= translate_y
# c_x = [init_x, (init_x+init_dx_dt/4), init_d2x_dt/12 + 2 * ((init_x+init_dx_dt/4)) - init_x, 0, target_x]
# c_y = [init_y, (init_y+init_dy_dt/4), init_d2y_dt/12 + 2 * ((init_y+init_dy_dt/4)) - init_y, 0, target_y]
# c = c_x + c_y
# initial = np.array(c)

track_horizon = np.array([*zip(track_points_x, track_points_y)])
line = geom.LineString(track_horizon)



c_x = [init_x, (init_x+init_dx_dt/5), init_d2x_dt/20 + 2 * ((init_x+init_dx_dt/5)) - init_x, init_x, init_x, target_x]
c_y = [init_y, (init_y+init_dy_dt/5), init_d2y_dt/20 + 2 * ((init_y+init_dy_dt/5)) - init_y, init_y, init_y, target_y]
c = c_x + c_y
initial = np.array(c)

print(c_x)
print(c_y)
# times = [t/point_horizon for t in range(point_horizon)]
# sp = SmoothBivariateSpline(times, track_points_x, track_points_y)


# def trajectory_x(cx, tau):
#     return (cx[0] * (1 - tau) ** 4) + (4 * cx[1] * tau * ((1 - tau) ** 3)) + (6 * cx[2] * (tau ** 2) * ((1 - tau) ** 2)) \
#            + (4 * cx[3] * (tau ** 3) * (1 - tau)) + cx[4] * tau ** 4
#     pass
#
#
# def trajectory_y(cy, tau):
#     return cy[0] * (1 - tau) ** 4 + cy[1] * 4 * tau * (1 - tau) ** 3 + cy[2] * 6 * (tau ** 2) * ((1 - tau) ** 2) \
#            + cy[3] * 4 * (tau ** 3) * ((1 - tau)) + cy[4] * tau ** 4
#     pass

def trajectory_x(cx, tau):
    return (cx[0] * (1 - tau) ** 5) + (5 * cx[1] * tau * ((1 - tau) ** 4)) + (10 * cx[2] * (tau ** 2) * ((1 - tau) ** 3)) \
           +(10 * cx[3] * (tau ** 3) * ((1 - tau) ** 2)) + (5 * cx[4] * (tau ** 4) * (1 - tau)) + (cx[5] * tau ** 5)
    pass


def trajectory_y(cy, tau):
    return (cy[0] * (1 - tau) ** 5) + (5 * cy[1] * tau * ((1 - tau) ** 4)) + (10 * cy[2] * (tau ** 2) * ((1 - tau) ** 3)) \
           + (10 * cy[3] * (tau ** 3) * ((1 - tau) ** 2)) + (5 * cy[4] * (tau ** 4) * (1 - tau)) + (cy[5] * tau ** 5)
    pass


# def speed_x(cx, tau):
#     return 4 * ((cx[1] - cx[0]) * (1 - tau) ** 3 + 3 * (cx[2] - cx[1]) * (1 - tau) ** 2 * tau
#                 + 3 * (cx[3] - cx[2]) * (1 - tau) * tau ** 2 + (cx[4] - cx[3]) * (tau) ** 3)
#     pass
#
#
# def speed_y(cy, tau):
#     return 4 * ((cy[1] - cy[0]) * (1 - tau) ** 3 + (cy[2] - cy[1]) * (1 - tau) ** 2 * tau
#                 + (cy[3] - cy[2]) * (1 - tau) * tau ** 2 + (cy[4] - cy[3]) * (tau) ** 3)
#     pass


def speed_x(cx, tau):
    return 5 * (((cx[1] - cx[0]) * (1 - tau) ** 4) + (4 * (cx[2] - cx[1]) * ((1 - tau) ** 3) * tau)
                + (6 * (cx[3] - cx[2]) * (1 - tau)**2 * (tau ** 2)) + (4*(cx[4] - cx[3]) * (1-tau)* ((tau) ** 3))
                + ((cx[5] - cx[4]) * tau**4))
    pass


def speed_y(cy, tau):
    return 5 * (((cy[1] - cy[0]) * (1 - tau) ** 4) + (4 * (cy[2] - cy[1]) * ((1 - tau) ** 3) * tau)
                + (6 * (cy[3] - cy[2]) * (1 - tau)**2 * (tau ** 2)) + (4*(cy[4] - cy[3]) * (1-tau)* ((tau) ** 3))
                + ((cy[5] - cy[4]) * tau**4))
    pass


# def acceleration_x(cx, tau):
#     return 12 * ((cx[2] - 2 * cx[1] + cx[0]) * (1 - tau) ** 2 + 2 * (cx[3] - 2 * cx[2] + cx[1]) * tau * (1 - tau)
#                  + (cx[4] - 2 * cx[3] + cx[2]) * tau ** 2)
#     pass
#
#
# def acceleration_y(cy, tau):
#     return 12 * ((cy[2] - 2 * cy[1] + cy[0]) * (1 - tau) ** 2 + 2 * (cy[3] - 2 * cy[2] + cy[1]) * tau * (1 - tau)
#                  + (cy[4] - 2 * cy[3] + cy[2]) * tau ** 2)
#     pass


def acceleration_x(cx, tau):
    return 20 * (((cx[2] - 2 * cx[1] + cx[0]) * (1 - tau) ** 3) + (3 * ((cx[3] - 2 * cx[2] + cx[1]) * tau * (1 - tau)**2))
                 + (3 * ((cx[4] - 2 * cx[3] + cx[2]) * tau**2 * (1 - tau))) + (cx[5] - 2 * cx[4] + cx[3]) * tau ** 3)
    pass


def acceleration_y(cy, tau):
    return 20 * (((cy[2] - 2 * cy[1] + cy[0]) * (1 - tau) ** 3) + (3 * ((cy[3] - 2 * cy[2] + cy[1]) * tau * (1 - tau)**2))
                 + (3 * ((cy[4] - 2 * cy[3] + cy[2]) * tau**2 * (1 - tau))) + (cy[5] - 2 * cy[4] + cy[3]) * tau ** 3)
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


# def distance_to_center(x, y, idx=None):
#     min_dist = np.inf
#     if idx is None:
#         for i in range(len(track_points_x)):
#             min_dist = min(dist(x, y, track_points_x[i],track_points_y[i]), min_dist)
#         return min_dist
#     else:
#         idx = max(idx, 0)
#         idx = min(idx, point_horizon-1)
#         return dist(x, y, track_points_x[idx], track_points_y[idx])
#     pass

def distance_to_center(x, y):
    point = geom.Point(x, y)
    return point.distance(line)

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

# print(arc_length(initial[:len(c_x)], initial[len(c_x):len(c_x)+len(c_y)]))
# # exit(0)
print(speed_x(initial[:len(c_x)], 0), speed_y(initial[len(c_x):len(c_x)+len(c_y)], 0), acceleration_x(initial[:len(c_x)], 0), acceleration_y(initial[len(c_x):len(c_x)+len(c_y)], 0))
# print(math.sqrt(coefficient_of_friction * gravitational_acceleration * radius(c[:len(c_x)], c[len(c_x):len(c_x)+len(c_y)], 0)))
# exit(1)
# opt = lambda c: - arc_length(c[:len(c_x)], c[len(c_x):len(c_x)+len(c_y)]) - math.sqrt(speed_x(c[:len(c_x)], 1)**2 + speed_y(c[len(c_x):len(c_x)+len(c_y)], 1)**2)

def opt(c):
    total = 0
    t = 0
    while t <= 1:
        total += distance_to_center(trajectory_x(c[:len(c_x)], t), trajectory_y(c[len(c_x):len(c_x)+len(c_y)], t))
        t += time_delta
    #             (c[len(c_x)+len(c_y)-1] - c[len(c_x)+len(c_y)-2]) * (main_track_y[position_index+point_horizon]-main_track_y[position_index+point_horizon-1])
    return total - arc_length(c[:len(c_x)], c[len(c_x):len(c_x)+len(c_y)])

constraints = []

# Speed Constraints
# t = 0
# while t <= 1:
#     # con = lambda c:  math.sqrt(speed_x(c[:len(c_x)], t)**2 + speed_y(c[len(c_x):len(c_x)+len(c_y)], t)**2) - math.sqrt(coefficient_of_friction * gravitational_acceleration * radius(c[:len(c_x)], c[len(c_x):len(c_x)+len(c_y)], t))
#     # nlc = NonlinearConstraint(con, -np.inf, 0)
#     con = lambda c:  math.sqrt(speed_x(c[:len(c_x)], t)**2 + speed_y(c[len(c_x):len(c_x)+len(c_y)], t)**2)
#     nlc = NonlinearConstraint(con, 0, 95)
#     # # print(t, con(initial))
#     constraints.append(nlc)
#     t += time_delta
A = []
for i in range(len(c_x)-1):
    A.append([0.0] * len(c))
    A[-1][i] = -1.0
    A[-1][i+1] = 1.0
for i in range(len(c_x), len(c_x) + len(c_y)-1):
    A.append([0.0] * len(c))
    A[-1][i] = -1.0
    A[-1][i+1] = 1.0
# print(A)
lc = LinearConstraint(A, 0, max_vel/5)
constraints.append(lc)

# Acceleration Constraints
# t = 0
# while t <= 1:
#     # con = lambda c:  math.sqrt(acceleration_x(c[:len(c_x)], t)**2 + acceleration_y(c[len(c_x):len(c_x)+len(c_y)], t)**2)
#     def con(c):
#         x_p = speed_x(c[:len(c_x)], t)
#         x_pp = acceleration_x(c[:len(c_x)], t)
#         y_p = speed_y(c[len(c_x):len(c_x)+len(c_y)], t)
#         y_pp = acceleration_y(c[len(c_x):len(c_x)+len(c_y)], t)
#         return (math.sqrt((x_p + x_pp * time_delta)**2 + (y_p + y_pp*time_delta)**2) - math.sqrt(x_p**2 + y_p**2))/time_delta
#     nlc = NonlinearConstraint(con, min_accel, max_accel)
#     constraints.append(nlc)
#     # print(t, con(initial))
#     t += time_delta
A = []
for i in range(len(c_x)-2):
    A.append([0.0] * len(c))
    A[-1][i] = 1.0
    A[-1][i+1] = -2.0
    A[-1][i+2] = 1.0
for i in range(len(c_x), len(c_x) + len(c_y)-2):
    A.append([0.0] * len(c))
    A[-1][i] = 1.0
    A[-1][i+1] = -2.0
    A[-1][i+2] = 1.0
# print(A)
lc = LinearConstraint(A, -7*gravitational_acceleration/20, 4*gravitational_acceleration/20)
constraints.append(lc)

# Track Bounds Constraints
# t = 0
# while t <= 1:
#     con = lambda c: distance_to_center(trajectory_x(c[:len(c_x)], t), trajectory_y(c[len(c_x):len(c_x)+len(c_y)], t))
#     # con = lambda c: dist(trajectory_x(c[:len(c_x)], t), trajectory_y(c[len(c_x):len(c_x)+len(c_y)], t), trajectory_x(c[:len(c_x)], t), sp.ev(t,trajectory_x(c[:len(c_x)], t)))
#     nlc = NonlinearConstraint(con, 0, (track_width/2)-car_width)
#     constraints.append(nlc)
#     # print(t, con(initial))
#     t += time_delta
pass

# Control Point Constraints
dy = target_y-init_y
dx = target_x-init_x
slope = dy/dx
if dy != 0:
    perp_slope = -1/slope
    target_inter = target_y - perp_slope * target_x
    init_inter = init_y - perp_slope * init_x
    lb = min(target_inter, init_inter)
    ub = max(target_inter, init_inter)
    A = []
    for i in range(3, len(c_x)-1):
        # con = lambda c: c[len(c_x) + i] - c[i] * perp_slope
        A.append([0.0] * len(c))
        A[-1][len(c_x) + i] = 1
        A[-1][i] = - perp_slope
    lc = LinearConstraint(A, lb, ub)
    constraints.append(lc)
else:
    lb = min(target_x, init_x)
    ub = max(target_x, init_x)
    A = []
    for i in range(3, len(c_x)-1):
        A.append([0] * len(c))
        # con = lambda c: c[i]
        A[-1][i] = 1
    lc = LinearConstraint(A, lb, ub)
    constraints.append(lc)

con = lambda c: c[len(c_x)-1] - main_track_x[int(c[-1])]
nlc = NonlinearConstraint(con, 0, 0)
constraints.append(nlc)
con = lambda c: c[len(c_x) + len(c_y)-1] - main_track_y[int(c[-1])]
nlc = NonlinearConstraint(con, 0, 0)
constraints.append(nlc)

# print(initial)
# bounds = Bounds([init_x, (init_x+init_dx_dt/4), init_d2x_dt/12 + 2 * ((init_x+init_dx_dt/4)) - init_x, 0, target_x-2,
#                  init_y, (init_y+init_dy_dt/4), init_d2y_dt/12 + 2 * ((init_y+init_dy_dt/4)) - init_y, 0, target_y-2],
#
#                 [init_x, (init_x+init_dx_dt/4), init_d2x_dt/12 + 2 * ((init_x+init_dx_dt/4)) - init_x, 600, target_x+2,
#                  init_y, (init_y+init_dy_dt/4), init_d2y_dt/12 + 2 * ((init_y+init_dy_dt/4)) - init_y, 800, target_y+2])


# bounds = Bounds([init_x, (init_x+init_dx_dt/5), init_d2x_dt/20 + 2 * ((init_x+init_dx_dt/5)) - init_x, min(init_x, target_x), min(init_x, target_x), target_x,
#                  init_y, (init_y+init_dy_dt/5), init_d2y_dt/20 + 2 * ((init_y+init_dy_dt/5)) - init_y, min(init_y, target_y), min(init_y, target_y), target_y],
#
#                 [init_x, (init_x+init_dx_dt/5), init_d2x_dt/20 + 2 * ((init_x+init_dx_dt/5)) - init_x, max(init_x, target_x), max(init_x, target_x), target_x,
#                  init_y, (init_y+init_dy_dt/5), init_d2y_dt/20 + 2 * ((init_y+init_dy_dt/5)) - init_y, max(init_y, target_y), min(init_y, target_y), target_y])



bounds = Bounds([init_x, (init_x+init_dx_dt/5), init_d2x_dt/20 + 2 * ((init_x+init_dx_dt/5)) - init_x, 0, 0, 0,
                 init_y, (init_y+init_dy_dt/5), init_d2y_dt/20 + 2 * ((init_y+init_dy_dt/5)) - init_y, 0, 0, 0],

                [init_x, (init_x+init_dx_dt/5), init_d2x_dt/20 + 2 * ((init_x+init_dx_dt/5)) - init_x, 800, 800, 800,
                 init_y, (init_y+init_dy_dt/5), init_d2y_dt/20 + 2 * ((init_y+init_dy_dt/5)) - init_y, 600, 600, 600])

result = minimize(opt, initial, bounds=bounds, constraints=constraints, options={'disp': True})
# result = basinhopping(opt, initial, minimizer_kwargs={'bounds':bounds, 'constraints':constraints})

# print(result.x)
# print(result)
# output = []
# for x in result.x[:len(c_x)]:
#     output.append(x + translate_x)
# for y in result.x[len(c_x): len(c_x) + len(c_y)]:
#     output.append(y+ translate_y)

output = [round(x, 3) for x in result.x]

print("c_x", output[:len(c_x)])
print("c_y", output[len(c_x):len(c_x)+len(c_y)])
print("trajectory distance:", arc_length(output[:len(c_x)], output[len(c_x):len(c_x)+len(c_y)]))
print("final speed", math.sqrt((speed_x(output[:len(c_x)], 1)**2) + (speed_y(output[len(c_x):len(c_x)+len(c_y)], 1)**2)))

# t = 0
# while t <= 1.05:
#     con = lambda c: distance_to_center(trajectory_x(c[:len(c_x)], t), trajectory_y(c[len(c_x):len(c_x)+len(c_y)], t))
#     # nlc = NonlinearConstraint(con, 0, (track_width/2)-car_width)
#     # constraints.append(nlc)
#     print("t=",t, "closest_distance_to_center_line=", con(output), "x=",trajectory_x(c[:len(c_x)], t),"y=", trajectory_y(c[len(c_x):len(c_x)+len(c_y)], t))
#     t += time_delta

t = 0
while t <= 1.05:
    con = lambda c:  math.sqrt(speed_x(c[:len(c_x)], t)**2 + speed_y(c[len(c_x):len(c_x)+len(c_y)], t)**2)
    print("time=", t, "speed=", con(output), "sp_x=", speed_x(output[:len(c_x)], t), "sp_y=", speed_y(output[len(c_x):len(c_x)+len(c_y)], t))
    t += time_delta

t = 0
while t <= 1.05:
    # con = lambda c:  math.sqrt(acceleration_x(c[:len(c_x)], t)**2 + acceleration_y(c[len(c_x):len(c_x)+len(c_y)], t)**2)
    def con(c):
        x_p = speed_x(c[:len(c_x)], t)
        x_pp = acceleration_x(c[:len(c_x)], t)
        y_p = speed_y(c[len(c_x):len(c_x)+len(c_y)], t)
        y_pp = acceleration_y(c[len(c_x):len(c_x)+len(c_y)], t)
        return (math.sqrt((x_p + x_pp * time_delta)**2 + (y_p + y_pp*time_delta)**2) - math.sqrt(x_p**2 + y_p**2))/time_delta
    print("time=", t, "acceleration=", con(output), "acc_x=", acceleration_x(output[:len(c_x)], t), "acc_y=", acceleration_y(output[len(c_x):len(c_x)+len(c_y)], t))
    t += time_delta


dy = target_y-init_y
dx = target_x-init_x
slope = dy/dx
if dy != 0:
    perp_slope = -1/slope
    target_inter = target_y - perp_slope * target_x
    init_inter = init_y - perp_slope * init_x
    lb = min(target_inter, init_inter)
    ub = max(target_inter, init_inter)
    for i in range(3, len(c_x)-1):
        con = lambda c: c[len(c_x) + i] - c[i] * perp_slope
        print(i, con(output)-1, lb, ub)
else:
    for i in range(3, len(c_x)-1):
        lb = min(target_x, init_x)
        ub = max(target_x, init_x)
        con = lambda c: c[i]
        print(i, con(output), lb, ub)


class Car():
    def __init__(self, cof, max_vel, max_accel, time_delta=0.001):
        self.cof = cof
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.track_index = 0
        self.x = 0.0
        self.y = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.d2x = 0.0
        self.d2y = 0.0
        self.actions = deque()
        self.visited_x=[]
        self.visited_y=[]
        self.time_delta=time_delta

    def progress_time(self):
        self.visited_x.append(self.x)
        self.visited_y.append(self.y)
        self.d2x, self.d2y = self.actions.popleft()
        self.x += self.time_delta*(self.dx + 0.5 * self.d2x*self.time_delta)
        self.y += self.time_delta*(self.dy + 0.5 * self.d2y*self.time_delta)
        self.dx += self.d2x*self.time_delta
        self.dy = self.d2y*self.time_delta


    def plan_motion(self, initial_x, initial_y, track_center):

        time = 0
        while time < 100:
            if math.isclose(math.fmod(time, 0.05), 0.0):
                # Plan horizon

                pass
            else:
                self.progress_time()
            time += self.time_delta
