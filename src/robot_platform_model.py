from simple_pid import PID
from dc_motor_model import dc_motor

# random platform geometry
WIDTH_DEFAULT=1
MASS_DEFAULT = 20 
MASS_R_DEFAULT = .5

# gear parameters
I_DEFAULT = 81

# motor parameters
# c42-l70-w10
R_DEFAULT = .62             # ohms
L_DEFAULT = 2e-3            # H
B_DEFAULT = 6.78e-7         # N * m / rad/s
K_DEFAULT = .275           # V / rad/s
J_DEFAULT = 1.4829e-3       # kg * m2


T_ = (J_DEFAULT * R_DEFAULT) / (K_DEFAULT * K_DEFAULT)
K_ = K_DEFAULT/(2*3.2258e-3)

# controller parameters
VOLTAGE_LIMITS_DEFAULT = 36
KP_DEFAULT = T_ * K_
KI_DEFAULT = K_
KD_DEFAULT = 0
DT_FACTOR = 100


class robot_platform():
    def __init__(self, *, width=WIDTH_DEFAULT, mass=MASS_DEFAULT, R=R_DEFAULT, L=L_DEFAULT, B=B_DEFAULT, J=J_DEFAULT, K=K_DEFAULT, dt):
        # motors parameters
        self.dc_motor_l = dc_motor(R=R, L=L, B=B, J_internal=J, K=K, dt=dt/DT_FACTOR) # ...
        self.dc_motor_r = dc_motor(R=R, L=L, B=B, J_internal=J, K=K, dt=dt/DT_FACTOR) # ...

        # inertial parameters
        self.width = width # ...
        self.mass = mass # ...

        # internal parameters
        self.l_wheel_pid = PID(Kp = KP_DEFAULT, Ki = KI_DEFAULT, Kd = KD_DEFAULT, setpoint=0, output_limits = (-VOLTAGE_LIMITS_DEFAULT, VOLTAGE_LIMITS_DEFAULT)) # ...
        self.r_wheel_pid = PID(Kp = KP_DEFAULT, Ki = KI_DEFAULT, Kd = KD_DEFAULT, setpoint=0, output_limits = (-VOLTAGE_LIMITS_DEFAULT, VOLTAGE_LIMITS_DEFAULT)) # ...
        self.l_wheel_speed = 0 # rad/s
        self.r_wheel_speed = 0 # rad/s

    def update_parameters(self, *, mass_ext=0):
        # тут надо считать и передавать на двигатели момент инерции
        if mass_ext:
            J_ext_ = mass_ext * MASS_R_DEFAULT**2 / I_DEFAULT**2
            self.dc_motor_l.update_J(J_ext=J_ext_)
            self.dc_motor_r.update_J(J_ext=J_ext_)
        else:
            self.dc_motor_l.update_J(J_ext=0)
            self.dc_motor_r.update_J(J_ext=0)
    
    # linear, angular
    def speed_cur(self):
        return (self.r_wheel_speed + self.l_wheel_speed)/2, (self.r_wheel_speed-self.l_wheel_speed)/self.width

    # linear, angular
    def speed_out(self, linear_target, angular_target):

        # target to wheels
        self.l_wheel_speed_target = (2*linear_target - angular_target*self.width)/2
        self.r_wheel_speed_target = (2*linear_target + angular_target*self.width)/2

        for cycle in range(DT_FACTOR):
            pass

            # wheels control
            self.l_wheel_voltage = self.l_wheel_pid(self.l_wheel_speed_target - self.dc_motor_l.get_speed())
            self.r_wheel_voltage = self.r_wheel_pid(self.r_wheel_speed_target - self.dc_motor_r.get_speed())
            print(f'l_wheel_voltage: {self.l_wheel_voltage}')
            print(f'r_wheel_voltage: {self.r_wheel_voltage}')

            # calculate wheels
            self.dc_motor_l.update(voltage=self.l_wheel_voltage)
            self.dc_motor_r.update(voltage=self.r_wheel_voltage)
            self.l_wheel_speed = self.dc_motor_l.get_speed()/I_DEFAULT
            self.r_wheel_speed = self.dc_motor_r.get_speed()/I_DEFAULT

        # wheels to output
        return self.speed_cur()
    

