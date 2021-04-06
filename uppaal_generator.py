import itertools
import math

import numpy as np
from lxml import etree

from car_models import Car, DiscreteInputModeCar
from car_modes import ControlType
from car_profiles import basicsports_profile, example_profile
from track import Track
from util import TPI
from pathos.multiprocessing import  ProcessingPool as Pool


def generate_mode_pairs(max_velocity, discrete_v_steps, discrete_h_steps=100):
    mode_pairs = []
    vel_precision = max_velocity/discrete_v_steps
    heading_precision = TPI/discrete_h_steps
    for v in np.arange(0, max_velocity+vel_precision, vel_precision):
        for h in np.arange(0, TPI, heading_precision):
            mode_pairs.append((v, h))
    return mode_pairs

def vh_name(velocity, heading):
    return f"v{str(round(velocity, 3)).replace('.', 'p')}h{str(round(heading, 3)).replace('.', 'p')}"

def generate_agent_template(agent: Car, id, num_steps, time_step):
    template = etree.Element("template")
    name = etree.SubElement(template, "name")
    name.text = f"Agent{id}"
    parameters = etree.SubElement(template, "parameter")
    parameters.text = "int id, double x, double y, double v, double h"
    declaration = etree.SubElement(template, "declaration")
    declaration.text = "hybrid clock goalTime;\n" \
                       "bool goal = false;\n" \
                        "void new_xy(double vi, double vf, double hi, double hf) \n\
                        { \n\
                            x = x + cos((hi + hf)/2)*((vi+vf)/2) * dt\n; \
                            y = y + sin((hi + hf)/2)*((vi+vf)/2) * dt;\n" \
                            "xs[id] = x;\n" \
                            "ys[id] = y;\n" \
                            "v = vf;\n" \
                            "h = hf;\n" \
                        "if (reached_goal(x,y)) { \n" \
                       "goal = true;\n" \
                       "goalTime = 0;\n" \
                       "}\n" \
                       "\
                        }\n"

    # Setup Modes
    vh_ids = {}
    vh_committed_ids = {}
    count = 2
    mps = generate_mode_pairs(agent.max_vel, agent.max_vel/agent.state.mode_manager.v_prec, TPI/agent.state.mode_manager.h_prec)
    print(len(mps))
    initial = None
    initial_transition_id = None
    for velocity, heading in mps:
        vx = int(velocity*math.cos(heading))
        vy = int(velocity*math.sin(heading))

        # Main location
        location = etree.SubElement(template, "location", id=f"id{count}", x=str(vx), y=str(vy))
        name = etree.SubElement(location, "name")
        name.text=vh_name(velocity, heading)
        invariant = etree.SubElement(location, "label", kind="invariant")
        invariant.text = "goalTime' == 0"
        vh_ids[(velocity,heading)] = f"id{count}"
        if math.isclose(agent.state.v, velocity) and math.isclose(agent.state.heading, heading):
            print("HERE")
            iloc = etree.SubElement(template, "location", id=f"id1")
            iname = etree.SubElement(iloc, "name")
            iname.text = "Initial"
            ilabel = etree.SubElement(iloc, "label", kind="invariant")
            ilabel.text = "goalTime' == 0"
            initial = etree.Element("init", ref=f"id1")
            initial_transition_id = count
        count += 1

        # Committed location
        location = etree.SubElement(template, "location", id=f"id{count}", x=str(vx), y=str(vy))
        name = etree.SubElement(location, "name")
        name.text = vh_name(velocity, heading) + "_c"
        invariant = etree.SubElement(location, "label", kind="invariant")
        invariant.text = "goalTime' == 0"
        etree.SubElement(location, "committed")
        vh_committed_ids[(velocity, heading)] = f"id{count}"
        count += 1

    # Setup Sink
    location = etree.SubElement(template, "location", id=f"id{count}")
    name = etree.SubElement(location, "name")
    invariant = etree.SubElement(location, "label", kind="invariant")
    invariant.text = "goalTime'==(dt*goal)"
    name.text = "Done"

    location = etree.SubElement(template, "location", id=f"id{count+1}")
    name = etree.SubElement(location, "name")
    name.text = "Done_c"
    etree.SubElement(location, "committed")


    if initial_transition_id is None or initial is None:
        print("couldn't set initial node")
        exit(1)

    # Setup Initial and Transition
    template.append(initial)
    transition = etree.SubElement(template, "transition")
    etree.SubElement(transition, "source", ref=f"id1")
    etree.SubElement(transition, "target", ref=f"id{initial_transition_id}")
    sync = etree.SubElement(transition, "label", kind="synchronisation")
    sync.text = f"Initialize?"

    def generate_transition_element_details(pair):
        p1 = pair[0]
        p2 = pair[1]
        m2c = {}
        c2m = {}
        # Mode to Committed Transition
        m2c['id1'] = vh_ids[p1]
        m2c['id2'] = vh_committed_ids[p1]
        m2c['sync_text'] = "chooseAgent?" if id == 0 else "chooseAdversary?"
        # Committed to Target Mode
        c2m['id1'] = vh_committed_ids[p1]
        c2m['id2'] = vh_ids[p2]
        c2m['guard_text'] = "main <= num_steps-1 && !goal"
        c2m['assignment_text'] = f"new_xy({p1[0]}, {p1[1]}, {p2[0]}, {p2[1]})"
        return m2c, c2m
    pool = Pool(processes=4)
    transitions = pool.map(generate_transition_element_details, filter(lambda pair: agent.state.mode_manager.is_transition_feasible(pair[0], pair[1], time_step),
                         itertools.combinations_with_replacement(mps, 2)))
    # for p1, p2 in filter(lambda pair: agent.state.mode_manager.is_transition_feasible(pair[0], pair[1], time_step),
    #                      itertools.combinations_with_replacement(mps, 2)):
    #     transition = etree.SubElement(template, "transition")
    #     etree.SubElement(transition, "source", ref=f"id{ids[p1]}")
    #     etree.SubElement(transition, "target", ref=f"{ids[p2]}")
    #     guard = etree.SubElement(transition, "label", kind="guard")
    #     guard.text = "dt >= 1 && main <= T - 1"
    #     assignment = etree.SubElement(transition, "label", kind="assignment")
    #     assignment.text = f"dt=0, new_xy({p1[0]}, {p1[1]}, {p2[0]}, {p2[1]})"

    # Setup Transitions within modes
    committed_finished = set()
    for transition_pair in transitions:
        m2c = transition_pair[0]
        if m2c['id1'] not in committed_finished:
            transition = etree.SubElement(template, "transition")
            etree.SubElement(transition, "source", ref=f"{m2c['id1']}")
            etree.SubElement(transition, "target", ref=f"{m2c['id2']}")
            sync = etree.SubElement(transition, "label", kind="synchronisation")
            sync.text = m2c['sync_text']
            committed_finished.add(m2c['id1'])
        c2m = transition_pair[1]
        transition = etree.SubElement(template, "transition")
        etree.SubElement(transition, "source", ref=f"{c2m['id1']}")
        etree.SubElement(transition, "target", ref=f"{c2m['id2']}")
        guard = etree.SubElement(transition, "label", kind="guard")
        guard.text = c2m['guard_text']
        assignment = etree.SubElement(transition, "label", kind="assignment")
        assignment.text = c2m['assignment_text']

    # Setup Transition to sink
    for mp in mps:
        transition = etree.SubElement(template, "transition")
        etree.SubElement(transition, "source", ref=vh_committed_ids[mp])
        etree.SubElement(transition, "target", ref=f"id{count}")
        guard = etree.SubElement(transition, "label", kind="guard")
        guard.text = "main>num_steps-1 || goal"

    transition = etree.SubElement(template, "transition")
    etree.SubElement(transition, "source", ref=f"id{count}")
    etree.SubElement(transition, "target", ref=f"id{count+1}")
    sync = etree.SubElement(transition, "label", kind="synchronisation")
    sync.text = "chooseAgent?" if id == 0 else "chooseAdversary?"

    transition = etree.SubElement(template, "transition")
    etree.SubElement(transition, "source", ref=f"id{count+1}")
    etree.SubElement(transition, "target", ref=f"id{count}")
    guard = etree.SubElement(transition, "label", kind="guard")
    guard.text = "main>num_steps-1 || goal"

    return template

def generate_track_template(agents):
    template = etree.Element("template")
    name = etree.SubElement(template, "name")
    name.text = f"Track"
    declaration = etree.SubElement(template, "declaration")
    declaration.text = "bool is_collide = false;\n" \
                       "void update_dist()\n" \
                       "{\n" \
                       "is_collide = false;\n" \
                       "min_dist = 99999.0;\n" \
                       "for (i = 0; i < num_players; i++){\n" \
                       "for (j=i+1; j < num_players; j++) {\n" \
                       "min_dist = fmin(hypot(xs[i]-xs[j], ys[i]-ys[j]), min_dist);\n" \
                       "if (fint(min_dist) <= 2)\n" \
                       "{\n" \
                       "is_collide = true;\n" \
                       "}\n" \
                       "}\n" \
                       "}\n" \
                       "}\n" \
                       "bool on_track = true;\n" \
                       "bool works = false;\n" \
                       "void is_on_track() {\n" \
                       "on_track = true;\n" \
                       "for (l = 0; l < num_players; l++){\n" \
                       "works = false;\n" \
                       "for (m = 0; m < track_points; m++) {\n" \
                       "works |= (fint(hypot(xs[l]-center_linex[m], ys[l]-center_liney[m])) <= fint(track_width));\n" \
                       "if (works == true) {\n" \
                       "m = track_points + 1;\n" \
                       "}\n" \
                       "}\n" \
                       "on_track &= works;\n" \
                       "if (on_track == false) {\n" \
                       "l = num_players + 1;\n" \
                       "}\n" \
                       "}\n" \
                       "}"

    initial = etree.Element("init", ref=f"id1")
    location = etree.SubElement(template, "location", id=f"id1")
    name = etree.SubElement(location, "name")
    name.text = "Initial"
    invar = etree.SubElement(location, "label", kind='invariant')
    invar.text = "starter <= 0"
    template.append(location)

    location = etree.SubElement(template, "location", id=f"id2")
    name = etree.SubElement(location, "name")
    etree.SubElement(location, "urgent")
    name.text = "AgentNext"
    template.append(location)

    location = etree.SubElement(template, "location", id=f"id3")
    name = etree.SubElement(location, "name")
    etree.SubElement(location, "urgent")
    name.text = "AdversaryNext"
    template.append(location)

    location = etree.SubElement(template, "location", id=f"id4")
    name = etree.SubElement(location, "name")
    etree.SubElement(location, "urgent")
    name.text = "Done"
    template.append(location)

    location = etree.SubElement(template, "location", id=f"id5")
    name = etree.SubElement(location, "name")
    name.text = "Wait"
    invar = etree.SubElement(location, "label", kind='invariant')
    invar.text = "waitTimer <= 1"
    template.append(location)

    location = etree.SubElement(template, "location", id=f"id6")
    name = etree.SubElement(location, "name")
    name.text = "Sink"
    template.append(location)
    template.append(initial)
    transition = etree.SubElement(template, "transition")
    etree.SubElement(transition, "source", ref="id1")
    etree.SubElement(transition, "target", ref=f"id2")
    assignment = etree.SubElement(transition, "label", kind="assignment")
    assignment.text = "main=0"
    sync = etree.SubElement(transition, "label", kind="synchronisation")
    sync.text = "Initialize!"
    template.append(transition)

    transition = etree.SubElement(template, "transition", controllable="false")
    etree.SubElement(transition, "source", ref="id2")
    etree.SubElement(transition, "target", ref=f"id3")
    guard = etree.SubElement(transition, "label", kind="guard")
    guard.text = "main <= num_steps"
    sync = etree.SubElement(transition, "label", kind="synchronisation")
    sync.text = "chooseAgent!"
    template.append(transition)

    transition = etree.SubElement(template, "transition", controllable="false")
    etree.SubElement(transition, "source", ref="id3")
    etree.SubElement(transition, "target", ref=f"id4")
    guard = etree.SubElement(transition, "label", kind="guard")
    guard.text = "main <= num_steps"
    sync = etree.SubElement(transition, "label", kind="synchronisation")
    sync.text = "chooseAdversary!"
    template.append(transition)

    transition = etree.SubElement(template, "transition", controllable="false")
    etree.SubElement(transition, "source", ref="id4")
    etree.SubElement(transition, "target", ref=f"id5")
    assignment = etree.SubElement(transition, "label", kind="assignment")
    assignment.text = "waitTimer=0"
    template.append(transition)

    transition = etree.SubElement(template, "transition", controllable="false")
    etree.SubElement(transition, "source", ref="id5")
    etree.SubElement(transition, "target", ref=f"id2")
    assignment = etree.SubElement(transition, "label", kind="assignment")
    assignment.text = "update_dist(), is_on_track()"
    guard = etree.SubElement(transition, "label", kind="guard")
    guard.text = "waitTimer == 1"
    template.append(transition)

    transition = etree.SubElement(template, "transition")
    etree.SubElement(transition, "source", ref="id2")
    etree.SubElement(transition, "target", ref=f"id6")
    guard = etree.SubElement(transition, "label", kind="guard")
    guard.text = "main > num_steps"
    template.append(transition)

    return template

def generate_uppaal_xml(agents, num_steps, time_step, track):
    root = etree.Element("nta")
    global_declaration = etree.SubElement(root,"declaration")
    global_declaration.text = f"clock waitTimer;\n" \
                              f"clock main;\n" \
                              f"clock starter;\n" \
                              f"const double dt = {time_step};\n" \
                              f"const int num_steps = {num_steps};\n" \
                              f"const int num_players = {len(agents)};\n" \
                              f"broadcast chan chooseAgent;\n" \
                              f"broadcast chan chooseAdversary;\n" \
                              f"broadcast chan Initialize;\n" \
                              f"broadcast chan update;\n" \
                              f"double min_dist = 5.0;\n" \
                              f"double xs[num_players];\n" \
                              f"double ys[num_players];\n" \
                              f"int i;\n" \
                              f"int j;\n" \
                              f"int l;\n" \
                              f"int m;\n" \
                              f"void update_dist()\n" \
                              "{\n" \
                              "min_dist = 99999.0;\n" \
                              "for (i = 0; i < num_players; i++){\n" \
                              "for (j=i+1; j<num_players; j++) {\n" \
                              "min_dist = fmin(hypot(xs[i]-xs[j], ys[i]-ys[j]), min_dist);\n" \
                              "}\n" \
                              "}\n" \
                              "}\n" \
                              "bool on_track = true;\n" \
                              f"const double track_width = {float(track.width/2)};\n" \
                              f"const int track_points = 100;\n" \
                              f"double center_linex[track_points] = {{{','.join(map(lambda x: str(float(x)), track.center_x[agents[0].state.tpx:agents[0].state.tpx+100]))}}};\n" \
                              f"double center_liney[track_points] = {{{','.join(map(lambda x: str(float(x)), track.center_y[agents[0].state.tpx:agents[0].state.tpx+100]))}}};\n" \
                              f"double dist_to_goal = 200.0;\n" \
                              f"bool reached_goal(double x, double y) \n" \
                              f"{{\n" \
                              f"return round(hypot(x-center_linex[track_points-1], y-center_liney[track_points-1])) <=3;\n" \
                              f"}}\n" \


    agent_decls = []
    agent_names = []
    for idx, agent in enumerate(agents):
        root.append(generate_agent_template(agent, idx, num_steps, time_step))
        decl = f"a{idx} = Agent{idx}({idx}, {agent.state.x}, {agent.state.y}, {agent.state.v}, {agent.state.heading});"
        agent_decls.append(decl)
        agent_names.append(f"a{idx}")

    root.append(generate_track_template(agents))
    system_declaration = etree.SubElement(root, "system")
    system_declaration.text = "\n".join(agent_decls) + "\n" + \
                              f"system {', '.join(agent_names)}, Track;"
    with open('output_file.xml', 'wb') as doc:
        doc.write(etree.tostring(root, pretty_print=True))


if __name__ == "__main__":
    control_params_1 = {
        'optimizer': None,
        'optimizer_params': {
            'min_point_horizon': 25,
            'max_point_horizon': 150,
            'bezier_order': 6,
            'plan_time_horizon': 5,
            'plan_time_precision': .1,
            'level': 1
        },
        'mode_manager_params': {
            'h_prec' : math.pi/4,
            'v_prec' : 0.5
        },
        'control_type': ControlType.STEER_ACCELERATE
    }
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
    car = DiscreteInputModeCar(67.0, 343.0, -.2, -.1, -.2, -.1, math.pi * (5/4), example_profile, track, control_params_1)
    car2 = DiscreteInputModeCar(67.0, 343.0, -.2, -.1, -.2, -.1, math.pi * (5/4), example_profile, track, control_params_1)
    generate_uppaal_xml([car, car2], 100, 0.5, track)
    pass