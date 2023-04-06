#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import numpy
import math
from random import randint as ri
from simple_pid import PID

PICTURE_PATH = '/Users/wtw/Desktop/pddmrcs/test_results/'
# PICTURE_PATH = /home/wtw-ub/workspace/pddmrcs/test_results/
RATE = 100
PLOT_PERIOD_TICKS = 20
TIME = 60

MAX_DIST = 4
MIN_DIST = .5
MAX_VEL_0 = .5
MAX_VEL = MAX_VEL_0
EPS = 1e-3

def goal_force_test(robot_pose, robot_vel, goal_pose, iteration = 0, *, coef_g = 1, mass = 1):
    max_linear_vel = MAX_VEL
    goal_dist_x = goal_pose[0] - robot_pose[0]
    goal_dist_y = goal_pose[1] - robot_pose[1]
    goal_dist = numpy.array([goal_dist_x, goal_dist_y, 0], numpy.dtype("float64"))
    goal_dist_norm = numpy.linalg.norm(goal_dist, ord=2)
    goal_dist_unit = numpy.array([0,0,0], numpy.dtype("float64"))
    if goal_dist_norm:
        goal_dist_unit = goal_dist / goal_dist_norm
    goal_force = coef_g * mass * (goal_dist_unit * max_linear_vel - robot_vel)
    if (not iteration%PLOT_PERIOD_TICKS) and (iteration >= (0*RATE)):
        # plot_it(robot_pose, goal_force, color='green', arrow=True)
        pass
    # print(f'goal force: {goal_force[0]: 3.3f} {goal_force[1]: 3.3f}')
    return goal_force

def social_force_test(robot_pose, robot_vel, agent_vel, agent_pose, iteration = 0, *, 
                      coef_a = 1,
                      coef_b = 1
                      ):
    actual_dist = robot_pose - agent_pose
    actual_dist_norm = numpy.linalg.norm(actual_dist, ord=2)
    global MAX_VEL
    vel_limit = MAX_VEL_0*(.1+math.exp(-((actual_dist_norm-MIN_DIST)**-1)/MAX_DIST))
    if actual_dist_norm < MIN_DIST:
        MAX_VEL = .1*MAX_VEL_0
    elif MAX_VEL > vel_limit:
        MAX_VEL = vel_limit
    chasing = (numpy.dot(agent_vel, robot_vel) > 0)
    robot_vel_norm = numpy.linalg.norm(robot_vel, ord=2)
    agent_vel_norm = numpy.linalg.norm(agent_vel)
    agent_vel_unit = numpy.array([0,0,0],numpy.dtype('float64'))
    if agent_vel_norm:
        agent_vel_unit = agent_vel/agent_vel_norm
    agent_to_robot_vel_x = agent_vel[0] - robot_vel[0]
    agent_to_robot_vel_y = agent_vel[1] - robot_vel[1]
    agent_to_robot_vel = numpy.array([agent_to_robot_vel_x, agent_to_robot_vel_y, 0], numpy.dtype("float64"))
    agent_to_robot_vel_norm = numpy.linalg.norm(agent_to_robot_vel, ord=2)
    if agent_to_robot_vel_norm < EPS:
        # plot_it(robot_pose, [0,0], color='red', aster=True)
        return numpy.array([0,0,0], numpy.dtype('float64'))
    agent_to_robot_vel_angle_cos = numpy.dot(actual_dist, agent_to_robot_vel) / (actual_dist_norm * agent_to_robot_vel_norm)
    if agent_to_robot_vel_angle_cos < 0: 
        # plot_it(robot_pose, [0,0], color='red', aster=True)
        return numpy.array([0,0,0], numpy.dtype('float64'))
    agent_to_robot_vel_unit = agent_to_robot_vel / agent_to_robot_vel_norm
    agent_to_robot_dist = actual_dist_norm * agent_to_robot_vel_angle_cos * agent_to_robot_vel_unit
    collision_dist = actual_dist - agent_to_robot_dist
    collision_dist_norm = numpy.linalg.norm(collision_dist, ord=2)
    if collision_dist_norm < EPS:
        collision_dist = (  EPS * 
                            numpy.cross(  robot_vel/robot_vel_norm, 
                                        numpy.array([0,0,1],numpy.dtype('float64'))) *
                                        (-1)**(chasing)
                            )
        collision_dist_norm = EPS
    agent_to_robot_dist_norm = numpy.linalg.norm(agent_to_robot_dist, ord=2)
    social_force = coef_a * agent_to_robot_dist_norm * math.exp( - collision_dist_norm / coef_b) * (collision_dist / collision_dist_norm)
    
    if chasing and (numpy.dot(agent_vel, social_force) > 0):
        social_force *= -1
        social_force = -social_force + 2*numpy.dot(agent_vel_unit, social_force)*agent_vel_unit
    elif not chasing and agent_vel_norm:
        # robot_vel_right_unit = (numpy.cross( robot_vel/robot_vel_norm, 
        #                         numpy.array([0,0,1],numpy.dtype('float64'))) *
        #                             (-1)**chasing)
        agent_vel_right_unit = (numpy.cross( agent_vel/agent_vel_norm, 
                                numpy.array([0,0,1],numpy.dtype('float64'))) *
                                    (-1)**chasing)
        if (numpy.linalg.norm(actual_dist-agent_vel_right_unit*EPS) > actual_dist_norm):
            social_force_norm = numpy.linalg.norm(social_force)
            agent_vel_to_force_sin = numpy.cross(agent_vel_unit, social_force)[2]/social_force_norm
            if (agent_vel_to_force_sin < 0):
                social_force = -social_force + 2*numpy.dot(agent_vel_unit, social_force)*agent_vel_unit
                pass

    if (not iteration%PLOT_PERIOD_TICKS) and (iteration >= (0*RATE)):
        # agent_vel_right_unit = (numpy.cross( agent_vel/agent_vel_norm, 
        #                         numpy.array([0,0,1],numpy.dtype('float64'))) *
        #                             (-1)**chasing)
        # plot_it(agent_pose, actual_dist, color='black', arrow=True)
        # plot_it(agent_pose, actual_dist-agent_vel_right_unit*.1, color='green', arrow=True)
        # plot_it(agent_pose, [0,agent_to_robot_vel_angle_cos], color='green')
        # plot_it(agent_pose, agent_to_robot_dist, color='purple')
        # plot_it(robot_pose, -collision_dist, color='orange')
        # plot_it(robot_pose, [social_force[0],social_force[1]], color='red', arrow=True)
        pass
    # print(f'social force: {social_force[0]: 3.3f} {social_force[1]: 3.3f}')
    return social_force

def obstacle_force_test(robot_pose, iteration=0, *, coef_o=1, coef_c=1):
    obstacle_force = numpy.array([0,0,0], numpy.dtype('float64'))
    wall_1 = -2
    wall_2 = 1
    if iteration == 0:
        plot_it(numpy.array([-4,wall_1,0]), numpy.array([8,0,0]), color='black')
        plot_it(numpy.array([-4,wall_2,0]), numpy.array([8,0,0]), color='black')
    obstacle_distance_1 = abs(robot_pose[1] - wall_1)
    obstacle_distance_2 = abs(robot_pose[1] - wall_2)
    obstacle_direction = coef_o * numpy.array([0,1,0], numpy.dtype('float64'))
    obstacle_force += math.exp(-obstacle_distance_1 / coef_c) * obstacle_direction
    obstacle_force += math.exp(-obstacle_distance_2 / coef_c) * (-obstacle_direction)
    if (not iteration%PLOT_PERIOD_TICKS) and (iteration >= (0*RATE)):
        # plot_it(robot_pose, obstacle_force, arrow=True, color='grey')
        pass
    return obstacle_force

def plot_it(origin, thing, *, color = 'purple', arrow=False, aster=False):
    plt.plot([origin[0], origin[0] + thing[0]], [origin[1], origin[1]+thing[1]], color=color)
    if arrow:
        if abs(thing[0])>EPS: 
            angle = math.atan(thing[1]/thing[0])-math.pi/2
        else: angle = math.pi/2 *(1 + (-1)**(thing[1]>0))
        if thing[0] < 0:
            angle += math.pi
        angle_deg = math.degrees(angle)
        plt.plot(origin[0] + thing[0], origin[1] + thing[1], color=color, marker=(3,0,angle_deg))
    elif aster: plt.plot(origin[0] + thing[0], origin[1] + thing[1], color=color, marker='*')

def rf(range = 3):
    return ri(-range,range-1) + ri(0,9)/10

def test_situation(*,
                    goal_pose = numpy.array([0, 0, 0], numpy.dtype("float64")),
                    robot_pose = numpy.array([0, 0, 0], numpy.dtype("float64")),
                    robot_vel = numpy.array([MAX_VEL, 0, 0], numpy.dtype("float64")),
                    agent_vel = numpy.array([MAX_VEL, 0, 0], numpy.dtype("float64")),
                    agent_pose = numpy.array([0, 0, 0], numpy.dtype("float64")),
                    _show = False,
                    _print = False,
                    _name = 'nothing',
                    coef_g = 1,
                    coef_a = 1,
                    coef_b = 1,
                        ):
    dt = RATE**(-1)
    _res = '_ne_uspel'
    array_robot_x = []
    array_robot_y = []
    array_agent_x = []
    array_agent_y = []
    social_force = numpy.array([0, 0, 0], numpy.dtype("float64"))
    if _print:
        plot_it(goal_pose, [0,0], aster=True, color = 'green')

    for i in range(TIME*RATE):
        array_robot_x.append(robot_pose[0])
        array_robot_y.append(robot_pose[1])
        array_agent_x.append(agent_pose[0])
        array_agent_y.append(agent_pose[1])
        goal_force = goal_force_test(robot_pose, robot_vel, goal_pose, i, coef_g = coef_g)
        social_force = social_force_test(robot_pose, robot_vel, agent_vel, agent_pose, i, coef_a=coef_a, coef_b=coef_b)
        sum_force = goal_force + social_force
        robot_vel += sum_force *dt
        robot_vel_norm = numpy.linalg.norm(robot_vel)
        if robot_vel_norm > MAX_VEL:
            robot_vel = MAX_VEL * (robot_vel/robot_vel_norm)
        robot_pose += robot_vel *dt
        agent_pose += agent_vel *dt
        if (numpy.linalg.norm(robot_pose - goal_pose) < MIN_DIST/2): 
            if _print: print(f'\nbreak on time = {i/RATE:3.3f}: goal reached')
            _res = '_ok'
            plot_it([robot_pose[0],robot_pose[1]],[robot_vel[0]*dt,robot_vel[1]*dt], color='blue', arrow=True)
            plot_it([agent_pose[0],agent_pose[1]],[agent_vel[0]*dt,agent_vel[1]*dt], color='magenta', arrow=True)
            break
        if (numpy.linalg.norm(robot_pose - agent_pose) < MIN_DIST): 
            if _print: print(f'\nbreak on time = {i/RATE:3.3f}: too close')
            _res = '_ploho'
            plot_it([robot_pose[0],robot_pose[1]],[robot_vel[0]*dt,robot_vel[1]*dt], color='blue', arrow=True)
            plot_it([agent_pose[0],agent_pose[1]],[agent_vel[0]*dt,agent_vel[1]*dt], color='magenta', arrow=True)
            break
    if not _print:
        plt.clf()
        return _res
    print('\nprinting...')
    plot_it([agent_pose[0],agent_pose[1]],[agent_vel[0]*dt,agent_vel[1]*dt], color='magenta', arrow=True)
    plot_it([robot_pose[0],robot_pose[1]],[robot_vel[0]*dt,robot_vel[1]*dt], color='blue', arrow=True)
    plt.plot(array_robot_x[0],array_robot_y[0], marker='o', color='blue')
    plt.plot(array_robot_x, array_robot_y, color='blue')
    plt.plot(array_agent_x[0],array_agent_y[0], marker='o', color='magenta')
    plt.plot(array_agent_x, array_agent_y, color='magenta')
    print('         ...done')
    plt.xlim((-4,4))
    plt.ylim((-4,4))
    plt.xlabel('x')
    plt.ylabel('y')
    plt.savefig(PICTURE_PATH + 
                _name + 
                _res + 
                '.png')
    if _show:
        plt.show()
    plt.clf()
    return _res

def test_situation_multi(*,
                    goal_pose = numpy.array([0, 0, 0], numpy.dtype("float64")),
                    robot_pose = numpy.array([0, 0, 0], numpy.dtype("float64")),
                    robot_vel = numpy.array([MAX_VEL, 0, 0], numpy.dtype("float64")),
                    agent_vel = [numpy.array([MAX_VEL, 0, 0], numpy.dtype("float64"))],
                    agent_pose = [numpy.array([0, 0, 0], numpy.dtype("float64"))],
                    _show = False,
                    _print = False,
                    _name = 'nothing',
                    coef_g = 1,
                    coef_a = 1,
                    coef_b = 1,
                    coef_o = 1, 
                    coef_c = 1,
                    mass = 1
                    ):
    
    global MAX_VEL
    dt = RATE**(-1)
    _res = '_ne_uspel'
    array_robot_x = []
    array_robot_y = []
    array_agent_x = []
    array_agent_y = []
    robot_path_length = 0
    for j in range(len(agent_pose)):
        array_agent_x.append([])
        array_agent_y.append([])
    if _print:
        plot_it(goal_pose, [0,0], aster=True, color = 'green')
    for i in range(TIME*RATE):
        social_force = numpy.array([0, 0, 0], numpy.dtype("float64"))
        array_robot_x.append(robot_pose[0])
        array_robot_y.append(robot_pose[1])
        MAX_VEL = MAX_VEL_0
        goal_force = goal_force_test(robot_pose, robot_vel, goal_pose, i, coef_g = coef_g, mass=mass)
        for j in range(len(agent_pose)):
            array_agent_x[j].append(agent_pose[j][0])
            array_agent_y[j].append(agent_pose[j][1])
            if numpy.linalg.norm(robot_pose - agent_pose[j]) < MAX_DIST:
                social_force += social_force_test(robot_pose, robot_vel, agent_vel[j], agent_pose[j], i, coef_a=coef_a, coef_b=coef_b)
        obstacle_force = obstacle_force_test(robot_pose, i,  coef_o=coef_o, coef_c=coef_c)
        if _print:
            if (not i%PLOT_PERIOD_TICKS):
                if numpy.linalg.norm(goal_force):
                    plot_it(robot_pose, goal_force, color='green', arrow=True)
                if numpy.linalg.norm(obstacle_force):
                    plot_it(robot_pose, obstacle_force, color='grey', arrow=True)
                if numpy.linalg.norm(social_force):
                    plot_it(robot_pose, [social_force[0],social_force[1]], color='red', arrow=True)
        sum_force = goal_force + social_force + obstacle_force
        robot_acc = sum_force/mass
        robot_vel_des = robot_vel+robot_acc*dt
        robot_vel_des_norm = numpy.linalg.norm(robot_vel_des)
        # print(MAX_VEL)
        if robot_vel_des_norm > MAX_VEL:
            robot_vel_des = MAX_VEL * (robot_vel_des/robot_vel_des_norm)
        robot_vel = robot_vel_des
        robot_pose += robot_vel*dt
        robot_path_length += numpy.linalg.norm(robot_vel)*dt
        for j in range(len(agent_pose)):
            agent_pose[j] += agent_vel[j] *dt
        if (numpy.linalg.norm(robot_pose - goal_pose) < MIN_DIST/2): 
            if _print: 
                print(f'\nbreak on time = {i/RATE:3.3f}: goal reached')
                print(f'total travel distance = {robot_path_length:3.3f}')
                print(f'robot_vel_norm: {numpy.linalg.norm(robot_vel):3.3f}')
            _res = '_ok'
            plot_it([robot_pose[0],robot_pose[1]],[robot_vel[0]*dt,robot_vel[1]*dt], color='blue', arrow=True)
            for j in range(len(agent_pose)):
                plot_it([agent_pose[j][0],agent_pose[j][1]],[agent_vel[j][0]*dt,agent_vel[j][1]*dt], color=['magenta', 'cyan'][j], arrow=True)
            break
        for j in range(len(agent_pose)):
            if (numpy.linalg.norm(robot_pose - agent_pose[j]) < MIN_DIST): 
                if _print: 
                    print(f'\nbreak on time = {i/RATE:3.3f}: too close')
                    print(f'robot_vel_norm: {numpy.linalg.norm(robot_vel):3.3f}')
                _res = '_ploho'
                plot_it([robot_pose[0],robot_pose[1]],[robot_vel[0]*dt,robot_vel[1]*dt], color='blue', arrow=True)
                plot_it([agent_pose[j][0],agent_pose[j][1]],[agent_vel[j][0]*dt,agent_vel[j][1]*dt], color=['magenta', 'cyan'][j], arrow=True)
                break
        if _res=='_ploho': break
                
    if not _print:
        plt.clf()
        return _res
    if _res == '_ne_uspel': print(f'\nbreak on time = {TIME:3.3f}: time limit')
    print('\nprinting...')
    for j in range(len(agent_pose)):
        plot_it([agent_pose[j][0],agent_pose[j][1]],[agent_vel[j][0]*dt,agent_vel[j][1]*dt], color=['magenta', 'cyan'][j], arrow=True)
        plt.plot(array_agent_x[j][0],array_agent_y[j][0], marker='o', color=['magenta', 'cyan'][j])
        plt.plot(array_agent_x[j], array_agent_y[j], color=['magenta', 'cyan'][j], label=f'agent {j} path')
    plot_it([robot_pose[0],robot_pose[1]],[robot_vel[0]*dt,robot_vel[1]*dt], color='blue', arrow=True)
    plt.plot(array_robot_x[0],array_robot_y[0], marker='o', color='blue')
    plt.plot(array_robot_x, array_robot_y, color='blue', label='robot path')
    print('         ...done')
    plt.xlim((-4,4))
    plt.ylim((-4,4))
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()

    plt.savefig(PICTURE_PATH + 
                _name + 
                _res + 
                '.png')
    if _show:
        plt.show()
    plt.clf()
    return _res

def nerand_test(*,
                    gp_x = 0, gp_y = 0,
                    rp_x = 0, rp_y = 0,
                    rv_x = 0, rv_y = 0,
                    ap_x = 0, ap_y = 0,
                    av_x = 0, av_y = 0,
                    coef_g,
                    coef_a,
                    coef_b
                    ):
    return test_situation(
                        goal_pose = numpy.array([gp_x, gp_y, 0], numpy.dtype("float64")),
                        robot_pose = numpy.array([rp_x, rp_y, 0], numpy.dtype("float64")),
                        robot_vel = numpy.array([rv_x, rv_y, 0], numpy.dtype("float64")),
                        agent_vel = numpy.array([av_x, av_y, 0], numpy.dtype("float64")),
                        agent_pose = numpy.array([ap_x, ap_y, 0], numpy.dtype("float64")),
                        _show = False,
                        _print = True,
                        _name = '_manual_test',
                        coef_g = coef_g,
                        coef_a = coef_a,
                        coef_b = coef_b
                            )

def nerand_test_multi(*,
                    gp_x = 0, gp_y = 0,
                    rp_x = 0, rp_y = 0,
                    rv_x = 0, rv_y = 0,
                    ap1_x = 0, ap1_y = 0,
                    av1_x = 0, av1_y = 0,
                    ap2_x = 0, ap2_y = 0,
                    av2_x = 0, av2_y = 0,
                    coef_g,
                    coef_a,
                    coef_b,
                    coef_o, 
                    coef_c,
                    m
                    ):
    return test_situation_multi(
                        goal_pose = numpy.array([gp_x, gp_y, 0], numpy.dtype("float64")),
                        robot_pose = numpy.array([rp_x, rp_y, 0], numpy.dtype("float64")),
                        robot_vel = numpy.array([rv_x, rv_y, 0], numpy.dtype("float64")),
                        agent_vel = [
                                        numpy.array([av1_x, av1_y, 0], numpy.dtype("float64")),
                                        numpy.array([av2_x, av2_y, 0], numpy.dtype("float64"))
                                        ],
                        agent_pose = [
                                        numpy.array([ap1_x, ap1_y, 0], numpy.dtype("float64")),
                                        numpy.array([ap2_x, ap2_y, 0], numpy.dtype("float64"))
                                        ],
                        _show = False,
                        _print = True,
                        _name = '_manual_test',
                        coef_g = coef_g,
                        coef_a = coef_a,
                        coef_b = coef_b,
                        coef_o = coef_o, 
                        coef_c = coef_c,
                        mass = m
                            )

def rand_tests( steps = 1, *,
                _print = False,
                coef_g = 1,
                coef_a = 3,
                coef_b = 5):
    cnt_ne_uspel = 0
    cnt_ok = 0
    cnt_ploho = 0
    for i in range(steps):
        if _print:
            print(f'testing {i=}')
        rp_x = rf(3)
        rp_y = rf(3)
        ap_x = rf(3)
        ap_y = rf(3)
        rv_x = MAX_VEL*rf(1)
        rv_y = (-1)**(ri(0,1))*math.sqrt(MAX_VEL**2 - rv_x**2)
        av_x = MAX_VEL*rf(1)
        av_y = (-1)**(ri(0,1))*math.sqrt(MAX_VEL**2 - av_x**2)
        res = test_situation(   goal_pose = numpy.array([0, 0, 0], numpy.dtype("float64")),
                                robot_pose = numpy.array([rp_x, rp_y, 0], numpy.dtype("float64")),
                                robot_vel = numpy.array([rv_x, rv_y, 0], numpy.dtype("float64")),
                                agent_vel = numpy.array([av_x, av_y, 0], numpy.dtype("float64")),
                                agent_pose = numpy.array([ap_x, ap_y, 0], numpy.dtype("float64")),
                                _name = 'rand_test_' + str(i),
                                _print = _print,
                                coef_g = coef_g,
                                coef_a = coef_a,
                                coef_b = coef_b
                                    )
        if res == '_ok':
            cnt_ok +=1
        elif res == '_ne_uspel':
            cnt_ne_uspel +=1
        elif res == '_ploho':
            cnt_ploho += 1
        pass
    
    print('\n')
    print(f'coef_g: {coef_g: 3.3f}, coef_a: {coef_a: 3.3f}, coef_b: {coef_b: 3.3f}')
    print(f'cnt_ok: {cnt_ok/steps: 3.3f}')
    print(f'cnt_ne_uspel: {cnt_ne_uspel/steps: 3.3f}')
    print(f'cnt_ploho: {cnt_ploho/steps: 3.3f}')
    # print('\n')
    return {'ok': cnt_ok/steps, 'ne uspel': cnt_ne_uspel/steps, 'ploho': cnt_ploho/steps}

def check(*, steps, coef_g,coef_a,coef_b):
    print(  rand_tests( steps,
                        _print=False,
                        coef_g=coef_g,
                        coef_a=coef_a,
                        coef_b=coef_b
                            )
            )

def look_for_optimal():
    filename = 'test_results.txt'
    print(f'printing to {filename}...')
    original_stdout = sys.stdout
    
    with open(filename, 'w') as f:
        try:
            for coef_1 in range(18,22,1):
                for coef_2 in range(15,25,1):
                    for coef_3 in range(15,25,1):
                        print(f'testing g={coef_1/10}, a={coef_2/10}, b={coef_3/10}')
                        sys.stdout = f
                        check(steps=100, coef_g=coef_1/10, coef_a=coef_2/10, coef_b=coef_3/10)
                        sys.stdout = original_stdout
        except KeyboardInterrupt:
            sys.stdout = original_stdout
            pass
    print(f'                              ...done')

def main():
    
    res = nerand_test_multi(gp_x = 3,           gp_y = -.5,
                            rp_x = -4,          rp_y = -.5,
                            rv_x = MAX_VEL,     rv_y = 0,
                            ap1_x = 3,          ap1_y = 0.,
                            av1_x = -MAX_VEL,   av1_y = 0.02,
                            ap2_x = 1,          ap2_y = -1.,
                            av2_x = -MAX_VEL,   av2_y = 0,
                            coef_g = .5,
                            coef_a = 1.5,
                            coef_b = 4.9,
                            coef_o = 5, 
                            coef_c = .3,
                            m = 10
                        )
    # print(res)
    pass

if __name__ == '__main__':
    main()




# useless fragments

    # look_for_optimal()
    
    # rand_tests(steps = 100, _print = True,
    #             coef_g = 2,
    #             coef_a = 2,
    #             coef_b = 2)

    # res = nerand_test(gp_x = 2, gp_y = -1,
    #                     rp_x = -3, rp_y = -1,
    #                     rv_x = MAX_VEL, rv_y = 0,
    #                     ap_x = 3, ap_y = 0,
    #                     av_x = -MAX_VEL/2, av_y = 0,
    #                     coef_g = 2,
    #                     coef_a = 2,
    #                     coef_b = 2
    #                     )

    # array_sum_force_x = []
    # array_sum_force_y = []
    # array_goal_force_x = []
    # array_goal_force_y = []
    # array_social_force_x = []
    # array_social_force_y = []
    # array_robot_vel_x = []
    # array_robot_vel_y = []
    # array_agent_vel_x = []
    # array_agent_vel_y = []
        # array_sum_force_x.append(sum_force[0])
        # array_sum_force_y.append(sum_force[1])
        # array_goal_force_x.append(goal_force[0])
        # array_goal_force_y.append(goal_force[1])
        # array_social_force_x.append(social_force[0])
        # array_social_force_y.append(social_force[1])
        # array_robot_vel_x.append(robot_vel[0])
        # array_robot_vel_y.append(robot_vel[1])
        # array_agent_vel_x.append(agent_vel[0])
        # array_agent_vel_y.append(agent_vel[1])
# for i in range(0, len(array_robot_x), PLOT_PERIOD_TICKS):
#     pass
    # pose correspondence
    # plt.plot([array_agent_x[i],array_robot_x[i]],[array_agent_y[i],array_robot_y[i]],color='orange')
    # goal force
    # plt.quiver(array_robot_x[i], array_robot_y[i], array_goal_force_x[i], array_goal_force_y[i], color='g', scale=3)
    # social force
    # plt.quiver(array_robot_x[i], array_robot_y[i], array_social_force_x[i], array_social_force_y[i], color='r', scale=3)
    # sum force
    # plt.quiver(array_robot_x[i], array_robot_y[i], array_sum_force_x[i], array_sum_force_y[i], color='b', scale=3)
    # speed
    # plt.quiver(array_robot_x[i], array_robot_y[i], array_robot_vel_x[i], array_robot_vel_y[i], color='b', scale=100)
    # plt.quiver(array_agent_x[i], array_agent_y[i], array_agent_vel_x[i], array_agent_vel_y[i], color='r', scale=100)


    # vel_pid_x = PID(Kp=0, Ki=10, Kd=0, setpoint=0)
    # vel_pid_y = PID(Kp=0, Ki=10, Kd=0, setpoint=0)
        # я не знаю, что с ними не так, поэтому вместо них будет масса
        # robot_vel[0] = -vel_pid_x(robot_vel_des[0]-robot_vel[0])
        # robot_vel[1] = -vel_pid_y(robot_vel_des[1]-robot_vel[1])