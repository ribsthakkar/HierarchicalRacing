import itertools
import math

import numpy as np
from lxml import etree

from car_models import Car, DiscreteInputModeCar
from car_modes import ControlType
from car_profiles import basicsports_profile, example_profile
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

def generate_agent_template(agent: Car, num_steps, time_step):
    template = etree.Element("template")
    name = etree.SubElement(template, "name")
    name.text = f"Agent"
    declaration = etree.SubElement(template, "declaration")
    declaration.text = "clock dt = 0.0;" \
                       "clock main = 0.0;" \
                       f"double v = {agent.state.v};"  \
                        f"double h = {agent.state.heading};"  \
                        f"double  x = {agent.state.x};"  \
                        f"double  y = {agent.state.y};"  \
                        "void new_xy(double vi, double vf, double hi, double hf) \
                        { \
                            x = x + cos((hi + hf)/2)*((vi+vf)/2) * rate; \
                            y = y + sin((hi + hf)/2)*((vi+vf)/2) * rate;" \
                        " \
                        }"

    # Setup Modes
    ids = {}
    count = 2
    mps = generate_mode_pairs(agent.max_vel, agent.max_vel/agent.state.mode_manager.v_prec, TPI/agent.state.mode_manager.h_prec)
    print(len(mps))
    initial = None
    initial_transition_id = None
    for velocity, heading in mps:
        vx = int(velocity*math.cos(heading))
        vy = int(velocity*math.sin(heading))
        location = etree.SubElement(template, "location", id=f"id{count}", x=str(vx), y=str(vy))
        name = etree.SubElement(location, "name")
        name.text=vh_name(velocity, heading)
        invariant = etree.SubElement(location, "label", kind="invariant")
        invariant.text = "dt <= 1 && main' == 1"
        ids[(velocity,heading)] = f"id{count}"
        if math.isclose(agent.state.v, velocity) and math.isclose(agent.state.heading, heading):
            iloc = etree.SubElement(template, "location", id=f"id1")
            iname = etree.SubElement(iloc, "name")
            iname.text = "Initial"
            ilabel = etree.SubElement(iloc, "label", kind="invariant")
            ilabel.text = "dt <= 0"
            initial = etree.Element("init", ref=f"id1")
            initial_transition_id = count
        count += 1
    if initial_transition_id is None or initial is None:
        print("couldn't set initial node")
        exit(1)

    # Setup Sink
    location = etree.SubElement(template, "location", id=f"id{count}")
    name = etree.SubElement(location, "name")
    name.text = "Sink"

    # Setup Initial and Transition
    template.append(initial)
    transition = etree.SubElement(template, "transition")
    etree.SubElement(transition, "source", ref=f"id1")
    etree.SubElement(transition, "target", ref=f"id{initial_transition_id}")
    guard = etree.SubElement(transition, "label", kind="guard")
    guard.text = "dt >= 0 && main <= T"
    assignment = etree.SubElement(transition, "label", kind="assignment")
    assignment.text = f"dt=0, main = 0"


    def generate_transition_element_details(pair):
        p1 =  pair[0]
        p2 = pair[1]
        result = {}
        result['id1'] = ids[p1]
        result['id2'] = ids[p2]
        result['guard_text'] = "dt >= 1 && main <= T - 1"
        result['assignment_text'] = f"dt=0, new_xy({p1[0]}, {p1[1]}, {p2[0]}, {p2[1]})"
        return result
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
    for t in transitions:
        transition = etree.SubElement(template, "transition")
        etree.SubElement(transition, "source", ref=f"{t['id1']}")
        etree.SubElement(transition, "target", ref=f"{t['id2']}")
        guard = etree.SubElement(transition, "label", kind="guard")
        guard.text = t['guard_text']
        assignment = etree.SubElement(transition, "label", kind="assignment")
        assignment.text = t['assignment_text']
        template.append(transition)

    # Setup Transition to sink
    for mp in mps:
        transition = etree.SubElement(template, "transition")
        etree.SubElement(transition, "source", ref=ids[mp])
        etree.SubElement(transition, "target", ref=f"id{count}")
        guard = etree.SubElement(transition, "label", kind="guard")
        guard.text = "main >= T"

    return template

def generate_track_template(agents):
    template = etree.Element("template")
    name = etree.SubElement(template, "name")
    name.text = f"Track"

    return template

def generate_uppaal_xml(agents, num_steps, time_step, track):
    root = etree.Element("nta")
    global_declaration = etree.SubElement(root,"declaration")
    global_declaration.text = f"const int num_steps = {num_steps};\n" \
                              f"const double rate = {time_step};" \


    for agent in agents:
        root.append(generate_agent_template(agent, num_steps, time_step))

    root.append(generate_track_template(agents))
    system_declaration = etree.SubElement(root, "system")
    system_declaration.text = "system  Agent;"
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
            'h_prec' : 0.1,
            'v_prec' : 0.5
        },
        'control_type': ControlType.STEER_ACCELERATE
    }
    car = DiscreteInputModeCar(100, 100, .2, .1, .2, .1, math.pi/2, example_profile, None, control_params_1)
    generate_uppaal_xml([car], 10, 0.5, None)
    pass