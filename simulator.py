import datetime
import itertools
import os
import math
import sys
from io import StringIO
from threading import Timer
from typing import List
from pathos.multiprocessing import  ProcessingPool as Pool
import matplotlib.pyplot as plt
plt.switch_backend('Qt5Agg')

from bezier_optimizer import bezier_race_optimize
from car_models import FourModeCar, Car, DiscreteInputModeCar
from car_modes import ControlType
from static_optimizer import static_race_optimize
from track import Track
from car_profiles import f1_profile, mclaren720s_profile, basicsports_profile
from util import rect_from_center, generate_heading_sweep

old_stdin = sys.stdin

class Simulator():
    def __init__(self, track: Track, cars: List[Car]):
        self.track = track
        self.cars = cars
        if len(cars) > 5 or len(cars) == 0:
            raise ValueError("The number of cars on track must be between 1 and 5")
        self.pool = Pool(processes=5)

    def _check_for_collisions(self, car_boxes, collision_tolerance=.5):
        collision = False
        for pair in itertools.combinations(car_boxes, r=2):
            if pair[0].intersection(pair[1]).area > collision_tolerance:
                car1_idx = car_boxes.index(pair[0])
                car2_idx = car_boxes.index(pair[1])
                print(f"COLLISION BETWEEN {self.cars[car1_idx]} AND {self.cars[car2_idx]}")
                collision = True
        return collision
    def simulate(self, time_step, update_frequency, total_steps, interactive, saving, interactive_after_steps=10,
                 update_visualization_after_steps=1, interactive_timeout=None):
        car_positions_x = {car: [] for car in self.cars}
        car_positions_y = {car: [] for car in self.cars}
        car_velocities = {car: [0] for car in self.cars}
        car_steering_angles = {car: [0] for car in self.cars}
        car_distances = {car: [0] for car in self.cars}
        colors = ['.b-', '.r-', '.c-', '.m-', '.y-']
        if saving:
            save_dir = os.path.join("game_theoretic_runs/", "game_theoretic_sim_" + str(datetime.datetime.now()))
            os.mkdir(save_dir)
        for i in range(1, total_steps+1):
            print(f"***---ROUND {i}---***")
            # actions = []
            # for car in self.cars:
            #     actions.append(car.plan_optimal_trajectory(self.cars, update_frequency, time_step))
            actions = self.pool.map(Car.plan_optimal_trajectory, self.cars,
                                    [list(filter(lambda c: c.state in self.track.cars_side[car] or c.state in self.track.cars_ahead[car], self.cars)) for car in self.cars],
                                    [update_frequency]*len(self.cars),
                                    [time_step]*len(self.cars))
            if saving or interactive:
                plt.figure(1)
                plt.plot(self.track.boundary1_x, self.track.boundary1_y, '.k-', label="Track Boundary Right")
                plt.plot(self.track.boundary2_x, self.track.boundary2_y, '.k-', label="Track Boundary Left")
                plt.plot(self.track.center_x, self.track.center_y, '.g-', label="Center Trajectory")
                plt.ion()
            t = 0
            while t <= (update_frequency) + time_step / 2:
                car_ordering = self.track.get_car_ordering()
                self.track.update_cars_ahead_side(update_frequency)
                for idx, car in enumerate(car_ordering):
                    initial_idx = self.cars.index(car)
                    print("CAR:", initial_idx, "Time: ", t)
                    if car_ordering[idx].get_control_type() == ControlType.STEER_ACCELERATE:
                        acceleration, steering, mode = actions[initial_idx].popleft()
                        acceleration, steering, mode = car_ordering[idx].input_steer_accelerate_command(acceleration, steering, mode, time_step)
                    elif car_ordering[idx].get_control_type() == ControlType.MODE_ONLY:
                        mode = actions[initial_idx].popleft()
                        acceleration, steering, mode = car_ordering[idx].input_mode_command(mode, time_step)
                    else:
                        print("unknown control type")
                        exit(1)
                    car_positions_x[car].append(car.state.x)
                    car_positions_y[car].append(car.state.y)
                    car_velocities[car].append(car.state.v)
                    car_steering_angles[car].append(steering * 180/math.pi)
                    car_distances[car].append(car_distances[car][-1] + time_step * math.sqrt(car.state.v))
                t += time_step
            car_boxes = [None] * len(self.cars)
            if saving or interactive:
                for idx, car in enumerate(self.cars):
                    initial_idx = self.cars.index(car)
                    plt.figure(1)
                    plt.plot(car_positions_x[car], car_positions_y[car], colors[initial_idx])
                    rect = rect_from_center(car.state.x, car.state.y, car.state.l, car.state.w, car. state.heading)
                    car_boxes[initial_idx] = rect
                    # sweep = generate_heading_sweep(car, update_frequency)
                    # plt.plot(*sweep.exterior.xy, colors[initial_idx][1:])
                    plt.plot(*rect.exterior.xy, colors[initial_idx][1:])
                    plt.annotate(f"Car {initial_idx} t={i*update_frequency}", (car_positions_x[car][-1], car_positions_y[car][-1]), fontsize=3)
                    if saving:
                        plt.savefig(save_dir + f"/round_{i}_position.png")
                    plt.figure(2)
                    plt.plot(car_distances[car], car_velocities[car], colors[initial_idx])
                    if saving:
                        plt.savefig(save_dir + f"/round_{i}_velocities.png")
                    plt.figure(3)
                    plt.plot(car_distances[car], car_steering_angles[car], colors[initial_idx])
                    if saving:
                        plt.savefig(save_dir + f"/round_{i}_steering_angles.png")
            collisions = self._check_for_collisions(car_boxes)
            if (interactive and (i % interactive_after_steps == 0)) or collisions:
                if interactive_timeout is not None and not collisions:
                    plt.draw()
                    plt.show()
                    sys.stdin = StringIO('Continuing...')
                    t = Timer(interactive_timeout, print, [""], {'file': sys.stdin})
                    t.start()
                    plt.pause(interactive_timeout + 5)
                    typed = input(f"Press [enter] to continue, or simulation will automatically continue in {interactive_timeout} seconds\n")
                    print(typed)
                    t.cancel()
                else:
                    plt.ioff()
                    plt.draw()
                    plt.show()
                    plt.ion()
                    input(f"Press [enter] to continue\n")
            elif (i % update_visualization_after_steps == 0):
                plt.draw()
                plt.show()
                plt.pause(.01)
        plt.plot(self.track.boundary1_x, self.track.boundary1_y, '.k-', label="Track Boundary Right")
        plt.plot(self.track.boundary2_x, self.track.boundary2_y, '.k-', label="Track Boundary Left")
        plt.plot(self.track.center_x, self.track.center_y, '.g-', label="Center Trajectory")
        for idx, car in enumerate(self.cars):
            plt.figure(1)
            plt.plot(car_positions_x[car], car_positions_y[car], colors[idx])
            plt.figure(2)
            plt.plot(car_distances[car], car_velocities[car], colors[idx])
            plt.figure(3)
            plt.plot(car_distances[car], car_steering_angles[car], colors[idx])
        plt.ioff()
        plt.draw()
        plt.show()


if __name__ == "__main__":
    main_track_x = (
    67, 65, 64, 64, 63, 62, 61, 61, 60, 60, 60, 59, 59, 59, 59, 59, 59, 59, 59, 59, 60, 60, 60, 61, 61, 62, 62, 63, 64,
    64, 65, 66, 66, 67, 68, 69, 69, 70, 71, 72, 73, 74, 74, 75, 76, 77, 78, 79, 79, 80, 81, 82, 82, 83, 84, 84, 85, 86,
    86, 87, 87, 88, 88, 88, 89, 89, 89, 90, 90, 90, 90, 90, 90, 90, 91, 91, 91, 91, 91, 91, 91, 90, 90, 90, 90, 90, 90,
    90, 90, 90, 89, 89, 89, 89, 89, 89, 88, 88, 88, 88, 88, 88, 88, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87,
    87, 87, 87, 88, 88, 88, 88, 89, 89, 89, 90, 90, 91, 91, 92, 92, 93, 93, 94, 95, 96, 96, 97, 98, 99, 100, 101, 102,
    103, 104, 105, 107, 108, 109, 110, 112, 113, 114, 116, 117, 119, 120, 122, 123, 125, 127, 128, 130, 132, 133, 135,
    137, 139, 141, 143, 144, 146, 148, 150, 152, 154, 156, 158, 160, 163, 165, 167, 169, 171, 173, 176, 178, 180, 182,
    185, 187, 189, 192, 194, 196, 199, 201, 204, 206, 208, 211, 213, 216, 218, 221, 223, 226, 228, 231, 233, 236, 239,
    241, 244, 246, 249, 251, 254, 257, 259, 262, 265, 267, 270, 272, 275, 278, 280, 283, 286, 288, 291, 293, 296, 299,
    301, 304, 307, 309, 312, 314, 317, 320, 322, 325, 327, 330, 332, 335, 338, 340, 343, 345, 348, 350, 353, 355, 358,
    360, 363, 365, 367, 370, 372, 375, 377, 379, 382, 384, 386, 389, 391, 393, 396, 398, 400, 402, 405, 407, 409, 411,
    413, 415, 418, 420, 422, 424, 426, 428, 430, 432, 434, 436, 438, 439, 441, 443, 445, 447, 449, 450, 452, 454, 456,
    457, 459, 460, 462, 464, 465, 467, 468, 470, 471, 472, 474, 475, 476, 478, 479, 480, 481, 483, 484, 485, 486, 487,
    488, 489, 490, 491, 492, 493, 494, 494, 495, 496, 497, 498, 498, 499, 500, 501, 501, 502, 503, 503, 504, 504, 505,
    506, 506, 507, 507, 508, 508, 509, 509, 510, 510, 511, 511, 512, 512, 513, 513, 514, 514, 514, 515, 515, 516, 516,
    517, 517, 518, 518, 518, 519, 519, 520, 520, 521, 521, 522, 522, 523, 523, 524, 524, 525, 526, 526, 527, 527, 528,
    529, 529, 530, 531, 531, 532, 533, 534, 534, 535, 536, 537, 538, 539, 539, 540, 541, 542, 543, 544, 545, 547, 548,
    549, 550, 551, 552, 554, 555, 556, 557, 559, 560, 561, 563, 564, 566, 567, 568, 570, 571, 573, 574, 576, 577, 579,
    580, 582, 584, 585, 587, 588, 590, 592, 593, 595, 597, 598, 600, 602, 604, 605, 607, 609, 611, 612, 614, 616, 618,
    619, 621, 623, 625, 627, 628, 630, 632, 634, 636, 638, 639, 641, 643, 645, 647, 649, 650, 652, 654, 656, 658, 659,
    661, 663, 665, 667, 668, 670, 672, 674, 676, 677, 679, 681, 683, 684, 686, 688, 689, 691, 693, 695, 696, 698, 699,
    701, 703, 704, 706, 707, 709, 711, 712, 714, 715, 717, 718, 720, 721, 722, 724, 725, 727, 728, 729, 731, 732, 733,
    734, 736, 737, 738, 739, 740, 741, 742, 743, 744, 746, 746, 747, 748, 749, 750, 751, 752, 753, 753, 754, 755, 755,
    756, 756, 757, 757, 758, 758, 758, 759, 759, 759, 759, 759, 759, 759, 759, 759, 759, 758, 758, 758, 757, 757, 756,
    755, 755, 754, 753, 752, 751, 750, 748, 747, 746, 744, 743, 741, 740, 738, 736, 734, 733, 731, 729, 727, 725, 723,
    721, 718, 716, 714, 712, 710, 707, 705, 703, 700, 698, 696, 693, 691, 689, 686, 684, 682, 679, 677, 675, 673, 670,
    668, 666, 664, 662, 660, 657, 655, 653, 651, 649, 647, 645, 643, 641, 640, 638, 636, 634, 632, 630, 628, 627, 625,
    623, 621, 620, 618, 616, 614, 613, 611, 609, 608, 606, 604, 603, 601, 599, 598, 596, 595, 593, 591, 590, 588, 587,
    585, 583, 582, 580, 578, 577, 575, 574, 572, 570, 569, 567, 565, 564, 562, 560, 559, 557, 555, 554, 552, 550, 549,
    547, 545, 544, 542, 540, 538, 537, 535, 533, 531, 530, 528, 526, 524, 523, 521, 519, 517, 515, 514, 512, 510, 508,
    506, 504, 503, 501, 499, 497, 495, 493, 491, 490, 488, 486, 484, 482, 480, 478, 476, 474, 472, 470, 468, 466, 465,
    463, 461, 459, 457, 455, 453, 451, 449, 447, 445, 443, 441, 439, 436, 434, 432, 430, 428, 426, 424, 422, 420, 418,
    416, 414, 412, 409, 407, 405, 403, 401, 399, 397, 394, 392, 390, 388, 386, 384, 381, 379, 377, 375, 372, 370, 368,
    366, 364, 361, 359, 357, 354, 352, 350, 348, 345, 343, 341, 338, 336, 334, 331, 329, 327, 325, 322, 320, 318, 315,
    313, 311, 308, 306, 304, 302, 299, 297, 295, 293, 290, 288, 286, 284, 282, 280, 277, 275, 273, 271, 269, 267, 265,
    263, 261, 259, 257, 255, 253, 251, 249, 247, 246, 244, 242, 240, 239, 237, 235, 234, 232, 231, 229, 228, 226, 225,
    223, 222, 221, 219, 218, 217, 216, 214, 213, 212, 211, 210, 209, 208, 207, 207, 206, 205, 204, 204, 203, 202, 202,
    201, 200, 200, 199, 199, 198, 198, 197, 197, 196, 196, 196, 195, 195, 195, 194, 194, 194, 193, 193, 193, 192, 192,
    192, 192, 191, 191, 191, 190, 190, 190, 190, 189, 189, 189, 188, 188, 188, 187, 187, 186, 186, 186, 185, 185, 184,
    184, 183, 183, 182, 181, 181, 180, 179, 179, 178, 177, 176, 175, 174, 173, 172, 171, 170, 169, 168, 167, 166, 164,
    163, 162, 160, 159, 157, 156, 154, 152, 151, 149, 147, 145, 143, 141, 139, 137, 135, 133, 131, 128, 126, 124, 121,
    119, 117, 115, 112, 110, 108, 105, 103, 101, 99, 96, 94, 92, 90, 88, 86, 84, 82, 80, 78, 77, 75, 73, 72, 70, 69, 68,
    66)
    main_track_y = (
    343, 343, 342, 341, 340, 339, 338, 336, 335, 334, 332, 331, 330, 328, 326, 325, 323, 322, 320, 318, 316, 314, 312,
    310, 309, 307, 304, 302, 300, 298, 296, 294, 291, 289, 287, 285, 282, 280, 278, 275, 273, 270, 268, 265, 263, 260,
    258, 255, 253, 250, 248, 245, 243, 240, 237, 235, 232, 230, 227, 224, 222, 219, 217, 214, 212, 209, 206, 204, 201,
    199, 196, 194, 191, 189, 186, 184, 181, 179, 176, 174, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 146,
    144, 141, 139, 137, 135, 133, 131, 129, 126, 124, 122, 120, 118, 117, 115, 113, 111, 109, 107, 106, 104, 102, 100,
    99, 97, 96, 94, 92, 91, 90, 88, 87, 85, 84, 83, 82, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 71, 70, 69, 69, 68, 67,
    67, 66, 66, 65, 65, 64, 64, 63, 63, 63, 63, 62, 62, 62, 62, 62, 61, 61, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62,
    62, 62, 62, 63, 63, 63, 63, 64, 64, 64, 65, 65, 65, 66, 66, 66, 67, 67, 67, 68, 68, 69, 69, 70, 70, 70, 71, 71, 72,
    72, 73, 73, 74, 74, 74, 75, 75, 76, 76, 77, 77, 78, 78, 78, 79, 79, 80, 80, 80, 81, 81, 81, 82, 82, 82, 83, 83, 83,
    84, 84, 84, 84, 85, 85, 85, 85, 85, 85, 85, 85, 85, 86, 86, 86, 85, 85, 85, 85, 85, 85, 85, 85, 85, 84, 84, 84, 84,
    84, 83, 83, 83, 83, 82, 82, 82, 82, 82, 81, 81, 81, 81, 80, 80, 80, 80, 79, 79, 79, 79, 79, 78, 78, 78, 78, 78, 78,
    78, 78, 77, 77, 77, 77, 77, 78, 78, 78, 78, 78, 78, 78, 79, 79, 79, 79, 80, 80, 81, 81, 82, 82, 83, 83, 84, 84, 85,
    86, 87, 88, 88, 89, 90, 91, 92, 94, 95, 96, 97, 98, 100, 101, 102, 104, 105, 107, 108, 110, 112, 113, 115, 116, 118,
    120, 122, 124, 125, 127, 129, 131, 133, 135, 137, 139, 141, 143, 145, 147, 149, 151, 153, 155, 158, 160, 162, 164,
    166, 169, 171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193, 195, 198, 200, 202, 205, 207, 209, 211, 214, 216,
    218, 220, 222, 225, 227, 229, 231, 233, 235, 238, 240, 242, 244, 246, 248, 250, 252, 254, 256, 258, 260, 262, 263,
    265, 267, 269, 271, 272, 274, 276, 277, 279, 281, 282, 284, 285, 287, 288, 290, 291, 293, 294, 296, 297, 298, 300,
    301, 302, 304, 305, 306, 308, 309, 310, 311, 312, 314, 315, 316, 317, 318, 319, 321, 322, 323, 324, 325, 326, 327,
    328, 329, 330, 331, 332, 333, 334, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351,
    352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 365, 366, 367, 368, 369, 370, 371, 372, 374, 375, 376,
    377, 378, 380, 381, 382, 384, 385, 386, 388, 389, 390, 392, 393, 394, 396, 397, 399, 400, 402, 403, 405, 407, 408,
    410, 411, 413, 415, 417, 418, 420, 422, 424, 426, 427, 429, 431, 433, 435, 437, 439, 442, 444, 446, 448, 450, 452,
    455, 457, 459, 461, 464, 466, 468, 471, 473, 475, 478, 480, 482, 484, 487, 489, 491, 493, 495, 497, 500, 502, 504,
    506, 508, 510, 511, 513, 515, 517, 519, 520, 522, 523, 525, 526, 527, 529, 530, 531, 532, 533, 534, 535, 535, 536,
    537, 537, 538, 538, 539, 539, 539, 539, 540, 540, 540, 540, 540, 540, 540, 540, 539, 539, 539, 538, 538, 538, 537,
    537, 536, 535, 535, 534, 534, 533, 532, 531, 530, 530, 529, 528, 527, 526, 525, 524, 523, 522, 521, 520, 519, 518,
    517, 515, 514, 513, 512, 511, 510, 509, 507, 506, 505, 504, 503, 502, 500, 499, 498, 497, 496, 495, 494, 492, 491,
    490, 489, 488, 487, 486, 485, 484, 483, 482, 482, 481, 480, 479, 478, 478, 477, 476, 476, 475, 474, 474, 473, 473,
    473, 472, 472, 472, 471, 471, 471, 471, 470, 470, 470, 470, 470, 470, 470, 470, 470, 470, 471, 471, 471, 471, 471,
    472, 472, 472, 473, 473, 473, 474, 474, 475, 475, 476, 476, 477, 477, 478, 479, 479, 480, 480, 481, 482, 483, 483,
    484, 485, 486, 486, 487, 488, 489, 490, 491, 491, 492, 493, 494, 495, 496, 497, 498, 499, 500, 501, 502, 503, 504,
    505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519, 520, 522, 523, 524, 525, 526, 527, 528,
    529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539, 540, 541, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551,
    551, 552, 553, 554, 555, 555, 556, 557, 558, 558, 559, 560, 560, 561, 561, 562, 562, 563, 563, 564, 564, 564, 564,
    565, 565, 565, 565, 565, 565, 565, 565, 565, 565, 565, 565, 564, 564, 564, 563, 563, 562, 562, 561, 560, 560, 559,
    558, 557, 556, 555, 554, 553, 552, 550, 549, 547, 546, 544, 543, 541, 539, 538, 536, 534, 532, 530, 528, 526, 524,
    522, 520, 518, 516, 514, 512, 510, 508, 506, 503, 501, 499, 497, 494, 492, 490, 488, 485, 483, 481, 478, 476, 474,
    471, 469, 467, 464, 462, 460, 457, 455, 453, 450, 448, 446, 443, 441, 439, 436, 434, 432, 430, 427, 425, 423, 421,
    419, 417, 414, 412, 410, 408, 406, 404, 402, 400, 398, 396, 394, 393, 391, 389, 387, 386, 384, 382, 381, 379, 377,
    376, 374, 373, 372, 370, 369, 368, 367, 365, 364, 363, 362, 361, 360, 360, 359, 358, 357, 357, 356, 356, 355, 355,
    354, 354, 354, 353, 353, 353, 353, 352, 352, 352, 352, 352, 351, 351, 351, 351, 351, 351, 351, 350, 350, 350, 350,
    349, 349, 349, 348, 348, 347, 347, 346, 346, 345, 344, 344)
    track_width = 10
    track = Track(main_track_x, main_track_y, track_width)
    control_params_1 = {
        'optimizer': bezier_race_optimize,
        'optimizer_params': {
            'min_point_horizon': 25,
            'max_point_horizon': 150,
            'bezier_order': 6,
            'plan_time_horizon': 5,
            'plan_time_precision': .1,
            'level': 1
        },
        'mode_manager_params': {
            'h_prec' : 0.01,
            'v_prec' : 0.5
        },
        'control_type': ControlType.STEER_ACCELERATE
    }
    all_cars = []
    car1 = track.place_car_of_type(DiscreteInputModeCar,x=67, y=343, dx=-.1, dy=-.1, d2x=-2, d2y=-2, heading=1.25*math.pi, car_profile=f1_profile, optimizer_parameters=control_params_1)
    all_cars.append(car1)
    car2 = track.place_car_of_type(DiscreteInputModeCar,x=60, y=341, dx=-.1, dy=-.1, d2x=-2, d2y=-2, heading=1.25*math.pi, car_profile=mclaren720s_profile, optimizer_parameters=control_params_1)
    all_cars.append(car2)
    control_params_2 = {
        'optimizer': bezier_race_optimize,
        'optimizer_params': {
            'min_point_horizon': 25,
            'max_point_horizon': 150,
            'bezier_order': 6,
            'plan_time_horizon': 5,
            'plan_time_precision': .1,
            'level': 0
        },
        'control_type': ControlType.STEER_ACCELERATE
    }
    car3 = track.place_car_of_type(DiscreteInputModeCar, x=60, y=335, dx=-.1, dy=-.1, d2x=-2, d2y=-2, heading=1.25*math.pi, car_profile=basicsports_profile, optimizer_parameters=control_params_2)
    all_cars.append(car3)
    simulator = Simulator(track, all_cars)
    simulator.simulate(time_step=0.1, update_frequency=0.5, total_steps=200, interactive=True, saving=False, interactive_after_steps=85, update_visualization_after_steps=1, interactive_timeout=None)