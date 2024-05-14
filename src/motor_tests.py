# from robot_platform_model import *
from dc_motor_model import dc_motor
from simple_pid import PID
import matplotlib.pyplot as plt
import numpy
import math


# random platform geometry
WIDTH_DEFAULT=1
MASS_DEFAULT = 20 
MASS_R_DEFAULT = .5

# gear parameters
I_DEFAULT = 0

# motor parameters
# c42-l70-w10
R_DEFAULT = .62             # ohms
L_DEFAULT = 2e-3            # H
B_DEFAULT = 6.78e-7         # N * m / rad/s
K_DEFAULT = .275           # V / rad/s
J_DEFAULT = 1.4829e-3       # kg * m2

# c23-l33-w10
# R_DEFAULT = 0.60            # ohms
# L_DEFAULT = 0.35e-3         # H
# B_DEFAULT = 9.549e-6
# K_DEFAULT = 0.0191         # V / rad/s
# J_DEFAULT = 1.554e-5        # kg * m2

T_ = (J_DEFAULT * R_DEFAULT) / (K_DEFAULT * K_DEFAULT) * 10 # *10 ?????
K_ = K_DEFAULT/(2*3.2258e-3) * 1                           

# controller parameters
VOLTAGE_LIMITS_DEFAULT = 36
KP_DEFAULT = T_ * K_
KI_DEFAULT = K_
KD_DEFAULT = 0

# DT_FACTOR = 1
DT_FACTOR = 10
RATE = int(1e4)
PRINTRATE = RATE/100
DT = RATE**-1
TIME = 1
EPS = 1e-3

_print = True
PICTURE_PATH = '/home/wtw-ub/workspace/pedsim/src/pddmrcs/src/tests/'
_show = False



def plot_it(origin, thing, *, color = 'purple', arrow=False, aster=False):
    # from matplotlib import pyplot as plt
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



def test():
    test_pid = PID( Kp = KP_DEFAULT, 
                    Ki = KI_DEFAULT, 
                    Kd = KD_DEFAULT, 
                    sample_time=DT, 
                    setpoint=0, 
                    # output_limits = (-VOLTAGE_LIMITS_DEFAULT, VOLTAGE_LIMITS_DEFAULT)
                    )   
    test_pid.reset()
    test_motor = dc_motor(  Ra=R_DEFAULT, 
                            La=L_DEFAULT, 
                            K=K_DEFAULT, 
                            B=B_DEFAULT, 
                            J_internal=J_DEFAULT, 
                            dt=DT/DT_FACTOR
                            )
    array_speed_target = []
    array_speed = []
    array_torque = []
    array_current = []
    array_time = []


    target_speed = 0

    for i in range(TIME*RATE):
        # target_speed = 100*math.sin(2*math.pi * TIME * i/RATE)                # sine
        target_speed = (i > TIME*RATE/2) * 100                                  # step

        target_current = -test_pid(target_speed - test_motor.get_speed())
        voltage_actual = 0
        for j in range(DT_FACTOR):
            if (target_current - test_motor.get_current() >= 0): voltage_actual = VOLTAGE_LIMITS_DEFAULT
            else: voltage_actual = -VOLTAGE_LIMITS_DEFAULT
            test_motor.update(voltage=voltage_actual)

        array_time.append(i/RATE)
        array_speed_target.append(target_speed)
        array_speed.append(test_motor.get_speed())
        array_torque.append(test_motor.get_torque())
        array_current.append(test_motor.get_current())
        if (not (i)%PRINTRATE and _print): 
            print(f'\ntime: {i/RATE}')
            print(f'target speed: {target_speed}')
            print(f'speed: {test_motor.get_speed()}')
            print(f'speed error: {target_speed - test_motor.get_speed()}')
            print(f'target current: {target_current}')  
            print(f'current: {test_motor.get_current()}')
            print(f'current error: {target_current - test_motor.get_current()}')  
            print(f'torque: {test_motor.get_torque()}') 
            # print(f'voltage: {voltage_}')
            print(f'voltage: {voltage_actual}')
    plt.plot(array_time, array_speed_target, color='cyan')
    plt.plot(array_time, array_speed, color='blue')
    plt.plot(array_time, array_current, color='red')
    plt.plot(array_time, array_torque, color='green')
    plt.xlim((0,TIME))
    plt.xlabel('time')
    plt.ylabel('w - blue, i - red, torque - green')

    _name = 'step'
    _res = ''    
    plt.savefig(PICTURE_PATH + 
                _name + 
                _res + 
                '.png')
    if _show:
        plt.show()

def main():
    test()
    return



if __name__ == '__main__':
    main()