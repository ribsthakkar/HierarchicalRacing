import datetime
from collections import deque
from functools import reduce, lru_cache

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
max_vel = 100
track_width = 10
car_width = 1.6
time_delta = 0.2
time_horizon = 5
main_track_x = (67, 65, 64, 64, 63, 62, 61, 61, 60, 60, 60, 59, 59, 59, 59, 59, 59, 59, 59, 59, 60, 60, 60, 61, 61, 62, 62, 63, 64, 64, 65, 66, 66, 67, 68, 69, 69, 70, 71, 72, 73, 74, 74, 75, 76, 77, 78, 79, 79, 80, 81, 82, 82, 83, 84, 84, 85, 86, 86, 87, 87, 88, 88, 88, 89, 89, 89, 90, 90, 90, 90, 90, 90, 90, 91, 91, 91, 91, 91, 91, 91, 90, 90, 90, 90, 90, 90, 90, 90, 90, 89, 89, 89, 89, 89, 89, 88, 88, 88, 88, 88, 88, 88, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 88, 88, 88, 88, 89, 89, 89, 90, 90, 91, 91, 92, 92, 93, 93, 94, 95, 96, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 107, 108, 109, 110, 112, 113, 114, 116, 117, 119, 120, 122, 123, 125, 127, 128, 130, 132, 133, 135, 137, 139, 141, 143, 144, 146, 148, 150, 152, 154, 156, 158, 160, 163, 165, 167, 169, 171, 173, 176, 178, 180, 182, 185, 187, 189, 192, 194, 196, 199, 201, 204, 206, 208, 211, 213, 216, 218, 221, 223, 226, 228, 231, 233, 236, 239, 241, 244, 246, 249, 251, 254, 257, 259, 262, 265, 267, 270, 272, 275, 278, 280, 283, 286, 288, 291, 293, 296, 299, 301, 304, 307, 309, 312, 314, 317, 320, 322, 325, 327, 330, 332, 335, 338, 340, 343, 345, 348, 350, 353, 355, 358, 360, 363, 365, 367, 370, 372, 375, 377, 379, 382, 384, 386, 389, 391, 393, 396, 398, 400, 402, 405, 407, 409, 411, 413, 415, 418, 420, 422, 424, 426, 428, 430, 432, 434, 436, 438, 439, 441, 443, 445, 447, 449, 450, 452, 454, 456, 457, 459, 460, 462, 464, 465, 467, 468, 470, 471, 472, 474, 475, 476, 478, 479, 480, 481, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 494, 495, 496, 497, 498, 498, 499, 500, 501, 501, 502, 503, 503, 504, 504, 505, 506, 506, 507, 507, 508, 508, 509, 509, 510, 510, 511, 511, 512, 512, 513, 513, 514, 514, 514, 515, 515, 516, 516, 517, 517, 518, 518, 518, 519, 519, 520, 520, 521, 521, 522, 522, 523, 523, 524, 524, 525, 526, 526, 527, 527, 528, 529, 529, 530, 531, 531, 532, 533, 534, 534, 535, 536, 537, 538, 539, 539, 540, 541, 542, 543, 544, 545, 547, 548, 549, 550, 551, 552, 554, 555, 556, 557, 559, 560, 561, 563, 564, 566, 567, 568, 570, 571, 573, 574, 576, 577, 579, 580, 582, 584, 585, 587, 588, 590, 592, 593, 595, 597, 598, 600, 602, 604, 605, 607, 609, 611, 612, 614, 616, 618, 619, 621, 623, 625, 627, 628, 630, 632, 634, 636, 638, 639, 641, 643, 645, 647, 649, 650, 652, 654, 656, 658, 659, 661, 663, 665, 667, 668, 670, 672, 674, 676, 677, 679, 681, 683, 684, 686, 688, 689, 691, 693, 695, 696, 698, 699, 701, 703, 704, 706, 707, 709, 711, 712, 714, 715, 717, 718, 720, 721, 722, 724, 725, 727, 728, 729, 731, 732, 733, 734, 736, 737, 738, 739, 740, 741, 742, 743, 744, 746, 746, 747, 748, 749, 750, 751, 752, 753, 753, 754, 755, 755, 756, 756, 757, 757, 758, 758, 758, 759, 759, 759, 759, 759, 759, 759, 759, 759, 759, 758, 758, 758, 757, 757, 756, 755, 755, 754, 753, 752, 751, 750, 748, 747, 746, 744, 743, 741, 740, 738, 736, 734, 733, 731, 729, 727, 725, 723, 721, 718, 716, 714, 712, 710, 707, 705, 703, 700, 698, 696, 693, 691, 689, 686, 684, 682, 679, 677, 675, 673, 670, 668, 666, 664, 662, 660, 657, 655, 653, 651, 649, 647, 645, 643, 641, 640, 638, 636, 634, 632, 630, 628, 627, 625, 623, 621, 620, 618, 616, 614, 613, 611, 609, 608, 606, 604, 603, 601, 599, 598, 596, 595, 593, 591, 590, 588, 587, 585, 583, 582, 580, 578, 577, 575, 574, 572, 570, 569, 567, 565, 564, 562, 560, 559, 557, 555, 554, 552, 550, 549, 547, 545, 544, 542, 540, 538, 537, 535, 533, 531, 530, 528, 526, 524, 523, 521, 519, 517, 515, 514, 512, 510, 508, 506, 504, 503, 501, 499, 497, 495, 493, 491, 490, 488, 486, 484, 482, 480, 478, 476, 474, 472, 470, 468, 466, 465, 463, 461, 459, 457, 455, 453, 451, 449, 447, 445, 443, 441, 439, 436, 434, 432, 430, 428, 426, 424, 422, 420, 418, 416, 414, 412, 409, 407, 405, 403, 401, 399, 397, 394, 392, 390, 388, 386, 384, 381, 379, 377, 375, 372, 370, 368, 366, 364, 361, 359, 357, 354, 352, 350, 348, 345, 343, 341, 338, 336, 334, 331, 329, 327, 325, 322, 320, 318, 315, 313, 311, 308, 306, 304, 302, 299, 297, 295, 293, 290, 288, 286, 284, 282, 280, 277, 275, 273, 271, 269, 267, 265, 263, 261, 259, 257, 255, 253, 251, 249, 247, 246, 244, 242, 240, 239, 237, 235, 234, 232, 231, 229, 228, 226, 225, 223, 222, 221, 219, 218, 217, 216, 214, 213, 212, 211, 210, 209, 208, 207, 207, 206, 205, 204, 204, 203, 202, 202, 201, 200, 200, 199, 199, 198, 198, 197, 197, 196, 196, 196, 195, 195, 195, 194, 194, 194, 193, 193, 193, 192, 192, 192, 192, 191, 191, 191, 190, 190, 190, 190, 189, 189, 189, 188, 188, 188, 187, 187, 186, 186, 186, 185, 185, 184, 184, 183, 183, 182, 181, 181, 180, 179, 179, 178, 177, 176, 175, 174, 173, 172, 171, 170, 169, 168, 167, 166, 164, 163, 162, 160, 159, 157, 156, 154, 152, 151, 149, 147, 145, 143, 141, 139, 137, 135, 133, 131, 128, 126, 124, 121, 119, 117, 115, 112, 110, 108, 105, 103, 101, 99, 96, 94, 92, 90, 88, 86, 84, 82, 80, 78, 77, 75, 73, 72, 70, 69, 68, 66)
main_track_y = (343, 343, 342, 341, 340, 339, 338, 336, 335, 334, 332, 331, 330, 328, 326, 325, 323, 322, 320, 318, 316, 314, 312, 310, 309, 307, 304, 302, 300, 298, 296, 294, 291, 289, 287, 285, 282, 280, 278, 275, 273, 270, 268, 265, 263, 260, 258, 255, 253, 250, 248, 245, 243, 240, 237, 235, 232, 230, 227, 224, 222, 219, 217, 214, 212, 209, 206, 204, 201, 199, 196, 194, 191, 189, 186, 184, 181, 179, 176, 174, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 146, 144, 141, 139, 137, 135, 133, 131, 129, 126, 124, 122, 120, 118, 117, 115, 113, 111, 109, 107, 106, 104, 102, 100, 99, 97, 96, 94, 92, 91, 90, 88, 87, 85, 84, 83, 82, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 71, 70, 69, 69, 68, 67, 67, 66, 66, 65, 65, 64, 64, 63, 63, 63, 63, 62, 62, 62, 62, 62, 61, 61, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62, 62, 62, 62, 63, 63, 63, 63, 64, 64, 64, 65, 65, 65, 66, 66, 66, 67, 67, 67, 68, 68, 69, 69, 70, 70, 70, 71, 71, 72, 72, 73, 73, 74, 74, 74, 75, 75, 76, 76, 77, 77, 78, 78, 78, 79, 79, 80, 80, 80, 81, 81, 81, 82, 82, 82, 83, 83, 83, 84, 84, 84, 84, 85, 85, 85, 85, 85, 85, 85, 85, 85, 86, 86, 86, 85, 85, 85, 85, 85, 85, 85, 85, 85, 84, 84, 84, 84, 84, 83, 83, 83, 83, 82, 82, 82, 82, 82, 81, 81, 81, 81, 80, 80, 80, 80, 79, 79, 79, 79, 79, 78, 78, 78, 78, 78, 78, 78, 78, 77, 77, 77, 77, 77, 78, 78, 78, 78, 78, 78, 78, 79, 79, 79, 79, 80, 80, 81, 81, 82, 82, 83, 83, 84, 84, 85, 86, 87, 88, 88, 89, 90, 91, 92, 94, 95, 96, 97, 98, 100, 101, 102, 104, 105, 107, 108, 110, 112, 113, 115, 116, 118, 120, 122, 124, 125, 127, 129, 131, 133, 135, 137, 139, 141, 143, 145, 147, 149, 151, 153, 155, 158, 160, 162, 164, 166, 169, 171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193, 195, 198, 200, 202, 205, 207, 209, 211, 214, 216, 218, 220, 222, 225, 227, 229, 231, 233, 235, 238, 240, 242, 244, 246, 248, 250, 252, 254, 256, 258, 260, 262, 263, 265, 267, 269, 271, 272, 274, 276, 277, 279, 281, 282, 284, 285, 287, 288, 290, 291, 293, 294, 296, 297, 298, 300, 301, 302, 304, 305, 306, 308, 309, 310, 311, 312, 314, 315, 316, 317, 318, 319, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 365, 366, 367, 368, 369, 370, 371, 372, 374, 375, 376, 377, 378, 380, 381, 382, 384, 385, 386, 388, 389, 390, 392, 393, 394, 396, 397, 399, 400, 402, 403, 405, 407, 408, 410, 411, 413, 415, 417, 418, 420, 422, 424, 426, 427, 429, 431, 433, 435, 437, 439, 442, 444, 446, 448, 450, 452, 455, 457, 459, 461, 464, 466, 468, 471, 473, 475, 478, 480, 482, 484, 487, 489, 491, 493, 495, 497, 500, 502, 504, 506, 508, 510, 511, 513, 515, 517, 519, 520, 522, 523, 525, 526, 527, 529, 530, 531, 532, 533, 534, 535, 535, 536, 537, 537, 538, 538, 539, 539, 539, 539, 540, 540, 540, 540, 540, 540, 540, 540, 539, 539, 539, 538, 538, 538, 537, 537, 536, 535, 535, 534, 534, 533, 532, 531, 530, 530, 529, 528, 527, 526, 525, 524, 523, 522, 521, 520, 519, 518, 517, 515, 514, 513, 512, 511, 510, 509, 507, 506, 505, 504, 503, 502, 500, 499, 498, 497, 496, 495, 494, 492, 491, 490, 489, 488, 487, 486, 485, 484, 483, 482, 482, 481, 480, 479, 478, 478, 477, 476, 476, 475, 474, 474, 473, 473, 473, 472, 472, 472, 471, 471, 471, 471, 470, 470, 470, 470, 470, 470, 470, 470, 470, 470, 471, 471, 471, 471, 471, 472, 472, 472, 473, 473, 473, 474, 474, 475, 475, 476, 476, 477, 477, 478, 479, 479, 480, 480, 481, 482, 483, 483, 484, 485, 486, 486, 487, 488, 489, 490, 491, 491, 492, 493, 494, 495, 496, 497, 498, 499, 500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519, 520, 522, 523, 524, 525, 526, 527, 528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539, 540, 541, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551, 551, 552, 553, 554, 555, 555, 556, 557, 558, 558, 559, 560, 560, 561, 561, 562, 562, 563, 563, 564, 564, 564, 564, 565, 565, 565, 565, 565, 565, 565, 565, 565, 565, 565, 565, 564, 564, 564, 563, 563, 562, 562, 561, 560, 560, 559, 558, 557, 556, 555, 554, 553, 552, 550, 549, 547, 546, 544, 543, 541, 539, 538, 536, 534, 532, 530, 528, 526, 524, 522, 520, 518, 516, 514, 512, 510, 508, 506, 503, 501, 499, 497, 494, 492, 490, 488, 485, 483, 481, 478, 476, 474, 471, 469, 467, 464, 462, 460, 457, 455, 453, 450, 448, 446, 443, 441, 439, 436, 434, 432, 430, 427, 425, 423, 421, 419, 417, 414, 412, 410, 408, 406, 404, 402, 400, 398, 396, 394, 393, 391, 389, 387, 386, 384, 382, 381, 379, 377, 376, 374, 373, 372, 370, 369, 368, 367, 365, 364, 363, 362, 361, 360, 360, 359, 358, 357, 357, 356, 356, 355, 355, 354, 354, 354, 353, 353, 353, 353, 352, 352, 352, 352, 352, 351, 351, 351, 351, 351, 351, 351, 350, 350, 350, 350, 349, 349, 349, 348, 348, 347, 347, 346, 346, 345, 344, 344)
def dist(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def calc_max_point_horizon(currx, curry, currdx, currdy, currd2x, currd2y, ipx):
    maxx = currdx + 0.5*currd2x * time_horizon**2
    maxy = currdy + 0.5*currd2y * time_horizon**2
    max_dist = math.sqrt(maxx**2 + maxy**2) * time_horizon
    for i in range(ipx+2, ipx+len(track_horizon)):
        # print(dist(track_horizon[i%len(track_horizon)][0], track_horizon[i%len(track_horizon)][1], currx, curry), max_dist)
        if dist(track_horizon[i%len(track_horizon)][0], track_horizon[i%len(track_horizon)][1], currx, curry) > max_dist:
            return i - ipx

def find_pos_index(init_px, currx, curry):
    min_dist = np.inf
    min_idx = 999999
    for i in range(init_px, init_px + 400):
        idx = i % len(main_track_x)
        d = dist(main_track_x[idx], main_track_y[idx], currx, curry)
        if d < min_dist:
            min_dist = d
            min_idx = i
    return min_idx


init_dx_dt = -6.422279423999987
init_dy_dt = -43.159778688
init_d2x_dt = -2.2091308800000204
init_d2y_dt = -12.558984959999952
init_position_index = 910
init_x = 188.51618150400003 #main_track_x[init_position_index]
init_y = 438.184094528 #main_track_y[init_position_index]
track_horizon = np.array([*zip(main_track_x, main_track_y)])
line = geom.LineString(track_horizon)

init_est_speed = math.sqrt(init_dx_dt**2 + init_dy_dt**2)

min_point_horizon = time_horizon*10 #int(max(1, init_est_speed))
max_point_horizon = 200 #calc_max_point_horizon(init_x, init_y, init_dx_dt, init_dy_dt, init_d2x_dt, init_d2y_dt, init_position_index)

bezier_order = 6

c1x = (init_x + time_horizon * init_dx_dt / bezier_order)
c1y = (init_y + time_horizon * init_dy_dt / bezier_order)
c2x = time_horizon ** 2 * init_d2x_dt / (bezier_order*(bezier_order-1)) + 2 * (c1x) - init_x
c2y = time_horizon ** 2 * init_d2y_dt / (bezier_order*(bezier_order-1)) + 2 * (c1y) - init_y

c_x = [init_x, c1x, c2x]
c_y = [init_y, c1y, c2y]

for i in range(3, bezier_order+1):
    if i-3 < (bezier_order-3)//2:
        c_x.append(c2x)
        c_y.append(c2y)
    elif i-3 == (bezier_order-3)//2:
        c_x.append((c2x + main_track_x[(init_position_index + min_point_horizon)%len(main_track_x)])/2)
        c_y.append((c2y + main_track_y[(init_position_index + min_point_horizon)%len(main_track_y)])/2)
    else:
        c_x.append(main_track_x[(init_position_index + min_point_horizon)%len(main_track_x)])
        c_y.append(main_track_y[(init_position_index + min_point_horizon)%len(main_track_y)])

lb = [init_x, c1x, c2x]
for i in range(3, bezier_order+1): lb.append(0)
lb += [init_y, c1y, c2y]
for i in range(3, bezier_order+1): lb.append(0)
lb.append(min_point_horizon)


ub = [init_x, c1x, c2x]
for i in range(3, bezier_order+1): ub.append(850)
ub += [init_y, c1y, c2y]
for i in range(3, bezier_order+1): ub.append(650)
ub.append(max_point_horizon)


c = c_x + c_y
c.append(min_point_horizon)
initial = np.array(c)

print("init controlx=", c_x)
print("init controly=", c_y)
print("min_pt_hz", min_point_horizon, "max_pt_hz", max_point_horizon)

@lru_cache(maxsize=None)
def comb(n,r):
    f = math.factorial
    return f(n) // f(r) // f(n-r)

def trajectory(c, tau):
    return reduce(lambda x,y: x+y,
                  map(lambda i: c[i] * comb(bezier_order, i) * ((1-tau/time_horizon)**(bezier_order-i)) * (tau/time_horizon)**i,
                      range(0, bezier_order+1)))

    # return (c[0] * (1 - (tau/time_horizon)) ** 5) + (5 * c[1] * (tau/time_horizon) * ((1 - (tau/time_horizon)) ** 4)) + (10 * c[2] * ((tau/time_horizon) ** 2) * ((1 - (tau/time_horizon)) ** 3)) \
    #        +(10 * c[3] * ((tau/time_horizon) ** 3) * ((1 - (tau/time_horizon)) ** 2)) + (5 * c[4] * ((tau/time_horizon) ** 4) * (1 - (tau/time_horizon))) + (c[5] * (tau/time_horizon) ** 5)

def speed(c, tau):
    return bezier_order/time_horizon * \
           reduce(lambda x, y: x + y,
                  map(lambda i: (c[i+1] - c[i]) * comb(bezier_order - 1, i) * ((1 - tau/time_horizon) ** (bezier_order - 1 - i)) * (tau/time_horizon) ** i,
                      range(0, bezier_order)))

    # return 5 * (((c[1] - c[0]) * (1 - (tau/time_horizon)) ** 4) + (4 * (c[2] - c[1]) * ((1 - (tau/time_horizon)) ** 3) * (tau/time_horizon))
    #             + (6 * (c[3] - c[2]) * (1 - (tau/time_horizon))**2 * ((tau/time_horizon) ** 2)) + (4*(c[4] - c[3]) * (1-(tau/time_horizon))* ((tau/time_horizon) ** 3))
    #             + ((c[5] - c[4]) * (tau/time_horizon)**4)) / time_horizon

def acceleration(c, tau):
    return (bezier_order*(bezier_order-1)) / time_horizon**2 * \
           reduce(lambda x, y: x + y,
                  map(lambda i: (c[i+2] - 2*c[i+1] + c[i]) * comb(bezier_order - 2, i) * ((1 - tau/time_horizon) ** (bezier_order - 2 - i)) * (tau/time_horizon) ** i,
                      range(0, bezier_order-1)))

    # return (20) * (((c[2] - 2 * c[1] + c[0]) * (1 - (tau/time_horizon)) ** 3) + (3 * ((c[3] - 2 * c[2] + c[1]) * (tau/time_horizon) * (1 - (tau/time_horizon))**2))
    #              + (3 * ((c[4] - 2 * c[3] + c[2]) * (tau/time_horizon)**2 * (1 - (tau/time_horizon)))) + (c[5] - 2 * c[4] + c[3]) * (tau/time_horizon) ** 3) /(time_horizon**2)

@lru_cache(maxsize=None)
def distance_to_center(x, y):
    point = geom.Point(x, y)
    return point.distance(line)

def arc_length(cx, cy, delta=0.05):
    tau1 = 0
    tau2 = tau1 + delta
    d = 0.0
    while tau1 < time_horizon:
        update = dist(trajectory(cx, tau2), trajectory(cy, tau2), trajectory(cx, tau1), trajectory(cy, tau1))
        d += update
        # print(tau2, trajectory(c_x, tau2), trajectory(c_y, tau2))
        # print(tau1, trajectory(cx, tau1), trajectory(cy, tau1))
        # print("t=",tau2, "speed=", update/delta)
        # if tau2 > 0.05:
        #     print("t=",tau2, "acc=", (update/delta-old_speed)/time_delta)
        tau1 = tau2
        tau2 += delta
        old_speed = update/delta
    return d

# print(arc_length(initial[:len(c_x)], initial[len(c_x):len(c_x)+len(c_y)]))
# # exit(0)
print(speed(initial[:len(c_x)], 0), speed(initial[len(c_x):len(c_x)+len(c_y)], 0), acceleration(initial[:len(c_x)], 0), acceleration(initial[len(c_x):len(c_x)+len(c_y)], 0))
# print(speed(initial[:len(c_x)], 1), speed(initial[len(c_x):len(c_x)+len(c_y)], 1), acceleration(initial[:len(c_x)], 1), acceleration(initial[len(c_x):len(c_x)+len(c_y)], 1))
# print(math.sqrt(coefficient_of_friction * gravitational_acceleration * radius(c[:len(c_x)], c[len(c_x):len(c_x)+len(c_y)], 0)))
# exit(1)
# opt = lambda c: - arc_length(c[:len(c_x)], c[len(c_x):len(c_x)+len(c_y)]) - math.sqrt(speed(c[:len(c_x)], time_horizon)**2 + speed(c[len(c_x):len(c_x)+len(c_y)], time_horizon)**2)

def get_acc_bound(sp):
    if 0 <= sp < 200/3.6:
        return 21 - (sp/(200/3.6))*10
    elif 200 / 3.6 <= sp < 300 / 3.6:
        return 10 - ((sp - 200/3.6) / (100/3.6))*5
    elif 300 / 3.6 <= sp <= 360 / 3.6:
        return 5 - ((sp - 300/3.6) / (60/3.6))*5
    else:
        return 1
    # return -0.1803 * sp + 21.008


def opt(c):
    total = 0
    t = 0
    while t <= time_horizon + time_delta / 2:
        if t > time_horizon - time_delta / 2: t = time_horizon
        x_p = speed(c[:len(c_x)], t)
        x_pp = acceleration(c[:len(c_x)], t)
        y_p = speed(c[len(c_x):len(c_x) + len(c_y)], t)
        y_pp = acceleration(c[len(c_x):len(c_x) + len(c_y)], t)
        sp = math.sqrt(x_p**2 + y_p**2)
        ac = math.sqrt(x_pp**2 + y_pp**2)
        def con3(c):
            if sp >= 0.1 and ac >= 0.1 and ((x_p * x_pp + y_p * y_pp) >= 0.01):
                return min(0, get_acc_bound(sp) - ac)
            else:
                return min(0, 5 * gravitational_acceleration - ac)

        total -= .8*con3(c)
        # total -= .5*ac/time_delta
        total -= .15*sp
        total += .8 * distance_to_center(trajectory(c[:len(c_x)], t), trajectory(c[len(c_x):len(c_x) + len(c_y)], t))
        # total -= find_pos_index(init_position_index, trajectory(c[:len(c_x)], t), trajectory(c[len(c_x):len(c_x)+len(c_y)], t))/time_delta
        t += time_delta
    #             (c[len(c_x)+len(c_y)-1] - c[len(c_x)+len(c_y)-2]) * (main_track_y[position_index+point_horizon]-main_track_y[position_index+point_horizon-1])
    # total += 5*math.sqrt((c[len(c_x)-1] - main_track_x[(int(c[-1]) + init_position_index)%len(main_track_x)])**2 + (c[len(c_x) + len(c_y)-1] - main_track_y[(int(c[-1]) + init_position_index)%len(main_track_y)])**2)
    # total += 10*math.sqrt((c[len(c_x) + len(c_y)-1] - main_track_y[(int(c[-1]) + init_position_index)%len(main_track_y)])**2)
    # total -= 20 * find_pos_index(init_position_index, trajectory(c[:len(c_x)], time_horizon), trajectory(c[len(c_x):len(c_x) + len(c_y)], time_horizon))
    return total

constraints = []

# Speed Constraints
t = 0
while t <= time_horizon+time_delta/2:
    if t > time_horizon-time_delta/2: t = time_horizon
    # con = lambda c:  math.sqrt(speed(c[:len(c_x)], t)**2 + speed(c[len(c_x):len(c_x)+len(c_y)], t)**2) - math.sqrt(coefficient_of_friction * gravitational_acceleration * radius(c[:len(c_x)], c[len(c_x):len(c_x)+len(c_y)], t))
    # nlc = NonlinearConstraint(con, -np.inf, 0)
    con = lambda c:  speed(c[:len(c_x)], t)**2 + speed(c[len(c_x):len(c_x)+len(c_y)], t)**2
    nlc = NonlinearConstraint(con, -np.inf, max_vel**2)
    # constraints.append(nlc)

    # con = lambda c:  acceleration(c[:len(c_x)], t)
    # nlc = NonlinearConstraint(con, -40, 21)
    # constraints.append(nlc)
    #
    # con = lambda c:  acceleration(c[len(c_x):len(c_x)+len(c_y)], t)
    # nlc = NonlinearConstraint(con, -40, 21)
    # constraints.append(nlc)

    t += time_delta

# # Acceleration Constraints
# t = 0
# while t <= time_horizon+time_delta/2:
#     if t > time_horizon-time_delta/2: t = time_horizon
#     def con(c):
#         x_p = speed(c[:len(c_x)], t)
#         x_pp = acceleration(c[:len(c_x)], t)
#         y_p = speed(c[len(c_x):len(c_x) + len(c_y)], t)
#         y_pp = acceleration(c[len(c_x):len(c_x) + len(c_y)], t)
#         sp = math.sqrt(x_p ** 2 + y_p ** 2)
#         ac = math.sqrt(x_pp ** 2 + y_pp ** 2)
#         if sp >= 0.01 and ((x_p * x_pp + y_p * y_pp)/(sp*ac) <= .08):
#             return 4*gravitational_acceleration - ac
#         else:
#             # if 0 <= sp <= 200/3.6: return 21 - ac
#             # elif 200/3.6 <= sp <= 300/3.6: return 10 - ac
#             # else: return 5 - ac
#             return -0.1803 * sp + 21.008 - math.sqrt(x_pp ** 2 + y_pp ** 2)
#     nlc = NonlinearConstraint(con, 0, np.inf)
#     constraints.append(nlc)
#     t += time_delta

# t = 0
# while t <= time_horizon+time_delta/2:
#     if t > time_horizon-time_delta/2: t = time_horizon
#     # con = lambda c:  math.sqrt(acceleration(c[:len(c_x)], t)**2 + acceleration(c[len(c_x):len(c_x)+len(c_y)], t)**2)
#     def con(c):
#         x_p = speed(c[:len(c_x)], t)
#         x_pp = acceleration(c[:len(c_x)], t)
#         y_p = speed(c[len(c_x):len(c_x)+len(c_y)], t)
#         y_pp = acceleration(c[len(c_x):len(c_x)+len(c_y)], t)
#         return (x_p*y_pp - y_p*x_pp)/math.sqrt(x_p**2 + y_p**2)
#     nlc = NonlinearConstraint(con, -5*gravitational_acceleration, 5*gravitational_acceleration)
#     constraints.append(nlc)
#     t += time_delta
# Track Bounds Constraints
t = 0
tot = 0
while t <= time_horizon+time_delta/2:
    if t > time_horizon-time_delta/2: t = time_horizon
    con = lambda c: distance_to_center(round(trajectory(c[:len(c_x)], t)), round(trajectory(c[len(c_x):len(c_x)+len(c_y)], t)))
    nlc = NonlinearConstraint(con, 0, (track_width/2)-car_width)
    constraints.append(nlc)
    t += time_delta
# con = lambda c: sum(map(lambda t: distance_to_center(round(trajectory(c[:len(c_x)], t*time_horizon)), round(trajectory(c[len(c_x):len(c_x)+len(c_y)], t*time_horizon))), range(int(time_horizon/time_delta))))
# nlc = NonlinearConstraint(con, -np.inf, ((track_width/2)-car_width) * (time_horizon/time_delta))
# constraints.append(nlc)

# Control Point Constraints
# dy = target_y-init_y
# dx = target_x-init_x
# slope = dy/dx
# if dy != 0:
#     perp_slope = -1/slope
#     target_inter = target_y - perp_slope * target_x
#     init_inter = init_y - perp_slope * init_x
#     lb = min(target_inter, init_inter)
#     ub = max(target_inter, init_inter)
#     A = []
#     for i in range(3, len(c_x)-1):
#         # con = lambda c: c[len(c_x) + i] - c[i] * perp_slope
#         A.append([0.0] * len(c))
#         A[-1][len(c_x) + i] = 1
#         A[-1][i] = - perp_slope
#     lc = LinearConstraint(A, lb, ub)
#     constraints.append(lc)
# else:
#     lb = min(target_x, init_x)
#     ub = max(target_x, init_x)
#     A = []
#     for i in range(3, len(c_x)-1):
#         A.append([0] * len(c))
#         # con = lambda c: c[i]
#         A[-1][i] = 1
#     lc = LinearConstraint(A, lb, ub)
#     constraints.append(lc)

# con = lambda c: c[-1] - init_position_index
# nlc = NonlinearConstraint(con, 0, 200)
# constraints.append(nlc)
con = lambda c: find_pos_index(init_position_index, trajectory(c[:len(c_x)], time_horizon), trajectory(c[len(c_x):len(c_x)+len(c_y)], time_horizon)) - init_position_index
nlc = NonlinearConstraint(con, min_point_horizon, max_point_horizon)
constraints.append(nlc)
# con = lambda c: (c[len(c_x) + len(c_y)-1] - main_track_y[(int(c[-1]) + init_position_index)%len(main_track_y)])**2
# nlc = NonlinearConstraint(con, 0, 3.4)
# constraints.append(nlc)
# min_x_target = min(main_track_x[(init_position_index+min_point_horizon)%len(main_track_x):(init_position_index+max_point_horizon)%len(main_track_x)])
# max_x_target = max(main_track_x[(init_position_index+min_point_horizon)%len(main_track_x):(init_position_index+max_point_horizon)%len(main_track_x)])
# min_y_target = min(main_track_y[(init_position_index+min_point_horizon)%len(main_track_y):(init_position_index+max_point_horizon)%len(main_track_y)])
# max_y_target = max(main_track_y[(init_position_index+min_point_horizon)%len(main_track_y):(init_position_index+max_point_horizon)%len(main_track_y)])
# print(min_x_target, max_x_target, min_y_target, max_y_target)
print("lb=", lb)
print("ub=", ub)
bounds = Bounds(lb, ub)
print(datetime.datetime.now())
# result = minimize(opt, initial, bounds=bounds, constraints=constraints, options={'disp': True, 'maxiter':2500})
result = basinhopping(opt, initial, niter=3, minimizer_kwargs={'bounds':bounds, 'constraints':constraints})
print(datetime.datetime.now())

# print(result.x)
# print(result)
# output = []

output = [round(x, 3) for x in result.x]

print("c_x", output[:len(c_x)])
print("c_y", output[len(c_x):len(c_x)+len(c_y)])
print("N", output[-1])
print("trajectory distance:", arc_length(output[:len(c_x)], output[len(c_x):len(c_x)+len(c_y)]))
print("final speed", math.sqrt((speed(output[:len(c_x)], time_horizon)**2) + (speed(output[len(c_x):len(c_x)+len(c_y)], time_horizon)**2)))
print("final speed x", speed(output[:len(c_x)], time_horizon), "final speed y", speed(output[len(c_x):len(c_x)+len(c_y)], time_horizon))
print("final acc x", acceleration(output[:len(c_x)], time_horizon), "final acc y", acceleration(output[len(c_x):len(c_x)+len(c_y)], time_horizon))

t=0
while t <= time_horizon+time_delta/2:
    if t > time_horizon-time_delta/2: t = time_horizon
    con = lambda c: distance_to_center(trajectory(c[:len(c_x)], t), trajectory(c[len(c_x):len(c_x)+len(c_y)], t))
    # nlc = NonlinearConstraint(con, 0, (track_width/2)-car_width)
    # constraints.append(nlc)
    x = trajectory(output[:len(c_x)], t)
    y = trajectory(output[len(c_x):len(c_x)+len(c_y)], t)
    print("t=",t, "closest_distance_to_center_line=", con(output), "x=",x,"y=",y,  "pos_index=", find_pos_index(init_position_index,x,y))
    t += time_delta
t=0
while t <= time_horizon+time_delta/2:
    if t > time_horizon-time_delta/2: t = time_horizon
    def con3(c):
        x_p = speed(c[:len(c_x)], t)
        x_pp = acceleration(c[:len(c_x)], t)
        y_p = speed(c[len(c_x):len(c_x) + len(c_y)], t)
        y_pp = acceleration(c[len(c_x):len(c_x) + len(c_y)], t)
        sp = math.sqrt(x_p ** 2 + y_p ** 2)
        ac = math.sqrt(x_pp ** 2 + y_pp ** 2)
        if sp >= 0.01 and ac >= 0.01 and ((x_p * x_pp + y_p * y_pp)/(sp*ac) <= .08):
            return 5*gravitational_acceleration - ac
        else:
            return get_acc_bound(sp) - ac


    x_p = speed(output[:len(c_x)], t)
    x_pp = acceleration(output[:len(c_x)], t)
    y_p = speed(output[len(c_x):len(c_x) + len(c_y)], t)
    y_pp = acceleration(output[len(c_x):len(c_x) + len(c_y)], t)
    sp = math.sqrt(x_p**2 + y_p**2)
    ac = math.sqrt(x_pp**2 + y_pp**2)
    print("time=", t, "speed=", sp, "sp_x=", speed(output[:len(c_x)], t), "sp_y=", speed(output[len(c_x):len(c_x)+len(c_y)], t))
    if sp >= 0.01 and ac >=0.01 and ((x_p * x_pp + y_p * y_pp)/(sp*ac) <= .08):
        print("time=", t, "acc_bound=", 5*gravitational_acceleration, "braking/turning_acc=", ac,"constraint3_val=", con3(output), "acc_x=", acceleration(output[:len(c_x)], t), "acc_y=", acceleration(output[len(c_x):len(c_x)+len(c_y)], t))
    else:
        print("time=", t, "acc_bound=", get_acc_bound(sp), "acc=", ac ,"constraint3_val=", con3(output), "acc_x=", acceleration(output[:len(c_x)], t), "acc_y=", acceleration(output[len(c_x):len(c_x)+len(c_y)], t))

    t += time_delta


# while t <= time_horizon+time_delta/2:
#     if t > time_horizon-time_delta/2: t = time_horizon
#     # con = lambda c:  math.sqrt(acceleration(c[:len(c_x)], t)**2 + acceleration(c[len(c_x):len(c_x)+len(c_y)], t)**2)
#     def con(c):
#         x_p = speed(c[:len(c_x)], t)
#         x_pp = acceleration(c[:len(c_x)], t)
#         y_p = speed(c[len(c_x):len(c_x)+len(c_y)], t)
#         y_pp = acceleration(c[len(c_x):len(c_x)+len(c_y)], t)
#         return (abs(x_p*y_pp - y_p*x_pp))/math.sqrt(x_pp**2 + y_pp**2)
#     print("time=", t, "lateral_acceleration=", con(output), "acc_x=", acceleration(output[:len(c_x)], t), "acc_y=", acceleration(output[len(c_x):len(c_x)+len(c_y)], t))
#     t += time_delta
#
#
# target_y = output[len(c_x)-1]
# target_x = output[len(c_x)+len(c_y)-1]
# dy = target_y-init_y
# dx = target_x-init_x
# slope = dy/dx
# if dy != 0:
#     perp_slope = -1/slope
#     target_inter = target_y - perp_slope * target_x
#     init_inter = init_y - perp_slope * init_x
#     lb = min(target_inter, init_inter)
#     ub = max(target_inter, init_inter)
#     for i in range(3, len(c_x)-1):
#         con = lambda c: c[len(c_x) + i] - c[i] * perp_slope
#         print(i, con(output)-1, lb, ub)
# else:
#     for i in range(3, len(c_x)-1):
#         lb = min(target_x, init_x)
#         ub = max(target_x, init_x)
#         con = lambda c: c[i]
#         print(i, con(output), lb, ub)

con = lambda c: c[len(c_x)-1] - main_track_x[(int(c[-1]) + init_position_index)%len(main_track_x)]
print("dist to actual", con(output), main_track_x[(int(output[-1]) + init_position_index)%len(main_track_x)])

con = lambda c: c[len(c_x) + len(c_y)-1] - main_track_y[(int(c[-1]) + init_position_index)%len(main_track_y)]
print("dist to actual", con(output), main_track_y[(int(output[-1]) + init_position_index)%len(main_track_y)])


con = lambda c: math.sqrt((c[len(c_x)-1] - main_track_x[(int(c[-1]) + init_position_index)%len(main_track_x)])**2 + (c[len(c_x) + len(c_y)-1] - main_track_y[(int(c[-1]) + init_position_index)%len(main_track_y)])**2)
print("dist to actual", con(output))


