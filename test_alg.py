#!/usr/bin/env python

import matplotlib.pyplot as plt
from control_server import *



def social_force_test(robot_pose, robot_vel, agent_vel, agent_pose):
    # input()

    # social_force = numpy.array([0,0,0], numpy.dtype("float64"))
    actual_dist = robot_pose - agent_pose
    actual_dist_norm = numpy.linalg.norm(actual_dist, ord=2)
    agent_vel_norm = numpy.linalg.norm(agent_vel, ord=2)
    robot_to_agent_vel_norm = numpy.linalg.norm(numpy.cross(robot_vel, agent_vel), ord=2) / agent_vel_norm
    agent_vel_unit = agent_vel / agent_vel_norm
    agent_vel_perp_unit = numpy.cross(agent_vel_unit, numpy.array([0,0,1], numpy.dtype("float64")))
    robot_to_agent_dir = (-1)**(numpy.dot(actual_dist, robot_vel) > 0)
    robot_to_agent_vel = agent_vel_perp_unit * robot_to_agent_vel_norm * robot_to_agent_dir
    agent_to_robot_vel_x = agent_vel[0] - robot_to_agent_vel[0]
    agent_to_robot_vel_y = agent_vel[1] - robot_to_agent_vel[1]
    agent_to_robot_vel = numpy.array([agent_to_robot_vel_x, agent_to_robot_vel_y, 0], numpy.dtype("float64"))
    agent_to_robot_vel_norm = numpy.linalg.norm(agent_to_robot_vel, ord=2)
    agent_to_robot_vel_angle_cos = numpy.dot(actual_dist, agent_to_robot_vel) / (actual_dist_norm * agent_to_robot_vel_norm)
    agent_to_robot_vel_unit = agent_to_robot_vel / agent_to_robot_vel_norm
    agent_to_robot_dist = actual_dist_norm * agent_to_robot_vel_angle_cos * agent_to_robot_vel_unit
    collision_time =  numpy.linalg.norm(agent_to_robot_dist / agent_to_robot_vel_norm, ord=2)
    collision_dist = actual_dist - agent_to_robot_dist
    collision_dist_norm = numpy.linalg.norm(collision_dist, ord=2)
    if collision_dist_norm < 1e-5: collision_dist_norm=1e-5
    coef_a = 1
    coef_b = 1
    social_force = coef_a * (robot_to_agent_vel_norm * robot_to_agent_dir / collision_time) * math.exp( - collision_dist_norm / coef_b) * (collision_dist / collision_dist_norm)

    print(f'actual_dist: {actual_dist}')
    print(f'agent_vel_norm: {agent_vel_norm}')
    print(f'robot_to_agent_vel_norm: {robot_to_agent_vel_norm}')
    print(f'agent_vel_perp_unit: {agent_vel_perp_unit}')
    print(f'robot_to_agent_vel: {robot_to_agent_vel}')
    print(f'agent_to_robot_vel: {agent_to_robot_vel}')
    print(f'agent_to_robot_vel_norm: {agent_to_robot_vel_norm}')
    print(f'agent_to_robot_vel_angle_cos: {agent_to_robot_vel_angle_cos}')
    print(f'agent_to_robot_dist: {agent_to_robot_dist}')
    print(f'collision_time: {collision_time}')
    print(f'social_force: {social_force}')
    
    robot_vel += social_force *dt
    robot_pose += robot_vel *dt
    agent_pose += agent_vel *dt
    print(f'robot_pose: {robot_pose}')
    print(f'agent_pose: {agent_pose}')
    print('\n')

dt = .01
robot_pose = numpy.array([0, 1, 0], numpy.dtype("float64"))
robot_vel = numpy.array([1, 0, 0], numpy.dtype("float64"))
agent_vel = numpy.array([-1, 1, 0], numpy.dtype("float64"))
agent_pose = numpy.array([2, 0, 0], numpy.dtype("float64"))
print(f'robot_pose: {robot_pose}')
print(f'agent_pose: {agent_pose}')
print('\n')

array_r_x = []
array_r_y = []
array_a_x = []
array_a_y = []
for i in range(100):
    social_force_test(robot_pose, robot_vel, agent_vel, agent_pose)
    array_r_x.append(robot_pose[0])
    array_r_y.append(robot_pose[1])
    array_a_x.append(agent_pose[0])
    array_a_y.append(agent_pose[1])
    if (numpy.linalg.norm(robot_pose - agent_pose) < .5): break

plt.plot(array_r_x, array_r_y)
plt.plot(array_a_x, array_a_y)
plt.xlim((0,3))
plt.ylim((0,3))
plt.savefig('/home/wtw-ub/workspace/pddmrcs/plt.png')