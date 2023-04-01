#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import numpy
import math
from random import randint as ri

PICTURE_PATH = '/Users/wtw/Desktop/pddmrcs/test_results/'
# PICTURE_PATH = /home/wtw-ub/workspace/pddmrcs/test_results/
RATE = 10
PLOT_PERIOD_TICKS = 200
TIME = 20
MIN_DIST = .5
MAX_VEL = .5
EPS = 1e-3

def goal_force_test(robot_pose, robot_vel, goal_pose, *, coef_g = 1):
    max_linear_vel = MAX_VEL
    goal_dist_x = goal_pose[0] - robot_pose[0]
    goal_dist_y = goal_pose[1] - robot_pose[1]
    goal_dist = numpy.array([goal_dist_x, goal_dist_y, 0], numpy.dtype("float64"))
    goal_dist_norm = numpy.linalg.norm(goal_dist, ord=2)
    goal_dist_unit = numpy.array([0,0,0], numpy.dtype("float64"))
    if goal_dist_norm:
        goal_dist_unit = goal_dist / goal_dist_norm
    goal_force = coef_g * (goal_dist_unit * max_linear_vel - robot_vel)
    # print(f'goal force: {goal_force[0]: 3.3f} {goal_force[1]: 3.3f}')
    return goal_force

def social_force_test(robot_pose, robot_vel, agent_vel, agent_pose, iteration = 0, *, 
                      coef_a = 1,
                      coef_b = 1
                      ):
    actual_dist = robot_pose - agent_pose
    actual_dist_norm = numpy.linalg.norm(actual_dist, ord=2)
    robot_vel_norm = numpy.linalg.norm(robot_vel, ord=2)
    agent_to_robot_vel_x = agent_vel[0] - robot_vel[0]
    agent_to_robot_vel_y = agent_vel[1] - robot_vel[1]
    agent_to_robot_vel = numpy.array([agent_to_robot_vel_x, agent_to_robot_vel_y, 0], numpy.dtype("float64"))
    agent_to_robot_vel_norm = numpy.linalg.norm(agent_to_robot_vel, ord=2)
    if agent_to_robot_vel_norm < EPS:
        # print('chasing?')
        return numpy.array([0,0,0], numpy.dtype('float64'))
    agent_to_robot_vel_angle_cos = numpy.dot(actual_dist, agent_to_robot_vel) / (actual_dist_norm * agent_to_robot_vel_norm)
    if agent_to_robot_vel_angle_cos < 0: 
        # print('not even close?')
        return numpy.array([0,0,0], numpy.dtype('float64'))
    agent_to_robot_vel_unit = agent_to_robot_vel / agent_to_robot_vel_norm
    agent_to_robot_dist = actual_dist_norm * agent_to_robot_vel_angle_cos * agent_to_robot_vel_unit
    collision_dist = actual_dist - agent_to_robot_dist
    collision_dist_norm = numpy.linalg.norm(collision_dist, ord=2)
    if collision_dist_norm < EPS: 
        collision_dist = EPS * numpy.cross(robot_vel/robot_vel_norm, 
                                            numpy.array([0,0,1],numpy.dtype('float64')))
        collision_dist_norm = EPS
    agent_to_robot_dist_norm = numpy.linalg.norm(agent_to_robot_dist, ord=2)
    social_force = coef_a * agent_to_robot_dist_norm * math.exp( - collision_dist_norm / coef_b) * (collision_dist / collision_dist_norm)
    if (not iteration%PLOT_PERIOD_TICKS) and (iteration >= (0*RATE)):
        # plot_it(agent_pose, agent_to_robot_dist, color='purple')
        # plot_it(robot_pose, -collision_dist, color='orange')
        plot_it(robot_pose, [social_force[0],social_force[1]], color='red', arrow=True)
        pass
    # print(f'social force: {social_force[0]: 3.3f} {social_force[1]: 3.3f}')
    return social_force

def plot_it(origin, thing, *, color = 'purple', arrow=False):
    plt.plot([origin[0], origin[0] + thing[0]], [origin[1], origin[1]+thing[1]], color=color)
    if arrow:
        angle = math.atan(thing[1]/thing[0])-math.pi/2
        if thing[0] < 0:
            angle += math.pi
        angle_deg = math.degrees(angle)
        plt.plot(origin[0] + thing[0], origin[1] + thing[1], color=color, marker=(3,0,angle_deg))

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
    for i in range(TIME*RATE):
        array_robot_x.append(robot_pose[0])
        array_robot_y.append(robot_pose[1])
        array_agent_x.append(agent_pose[0])
        array_agent_y.append(agent_pose[1])
        goal_force = goal_force_test(robot_pose, robot_vel, goal_pose, coef_g = coef_g)
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
    plot_it([robot_pose[0],robot_pose[1]],[robot_vel[0]*dt,robot_vel[1]*dt], color='blue', arrow=True)
    plot_it([agent_pose[0],agent_pose[1]],[agent_vel[0]*dt,agent_vel[1]*dt], color='magenta', arrow=True)
    plt.plot(array_robot_x[0],array_robot_y[0], marker='o', color='blue')
    plt.plot(array_robot_x, array_robot_y, color='blue')
    plt.plot(array_agent_x[0],array_agent_y[0], marker='o', color='magenta')
    plt.plot(array_agent_x, array_agent_y, color='magenta')
    print('         ...done')
    plt.xlim((-4,4))
    plt.ylim((-4,4))
    plt.xlabel('x')
    plt.ylabel('y')
    plt.savefig(PICTURE_PATH + _name + _res + '.png')
    if _show:
        plt.show()
    plt.clf()
    return _res

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
            for coef_1 in range(6,12,1):
                for coef_2 in range(15,25,2):
                    for coef_3 in range(20,40,5):
                        print(f'testing g={coef_1/10}, a={coef_2/10}, b={coef_3/10}')
                        sys.stdout = f
                        check(steps=100, coef_g=coef_1/10, coef_a=coef_2/10, coef_b=coef_3/10)
                        sys.stdout = original_stdout
        except KeyboardInterrupt:
            sys.stdout = original_stdout
            pass
    print(f'                              ...done')


def main():
    # look_for_optimal()
    # -> coef_g:  1.100, coef_a:  2.000, coef_b:  3.500
    # -> coef_g:  0.800, coef_a:  2.000, coef_b:  2.500

    # rand_tests(steps = 10, _print = True,
    #             coef_g = 1.1,
    #             coef_a = 2,
    #             coef_b = 3.5)

    # res = nerand_test(      gp_x = 3, gp_y = 0,
    #                         rp_x = -3, rp_y = 0.01,
    #                         ap_x = -1, ap_y = 0,
    #                         rv_x = MAX_VEL, rv_y = 0,
    #                         av_x = MAX_VEL/10, av_y = 0,
    #                         coef_g = 1.1,
    #                         coef_a = 2,
    #                         coef_b = 3.5
    #                     )
    # print(res)
    pass

if __name__ == '__main__':
    main()




# useless fragments


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