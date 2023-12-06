from simple_pid import PID
from dc_motor_model import dc_motor


class robot_platform():
    def __init__(self, *, width=1, R, L, B, Kt, J, Kb, dt):
        # motors parameters
        self.dc_motor_l = dc_motor(R, L, B, Kt, J, Kb, dt) # ...
        self.dc_motor_r = dc_motor(R, L, B, Kt, J, Kb, dt) # ...

        # inertial parameters
        self.width = width # ...

        # internal parameters
        self.l_wheel_pid = PID(Kp = 1, Ki = 0, Kd = 0, setpoint=0, output_limits = (-0, 0)) # ...
        self.r_wheel_pid = PID(Kp = 1, Ki = 0, Kd = 0, setpoint=0, output_limits = (-0, 0)) # ...
        self.l_wheel_speed = 0 # rad/s
        self.r_wheel_speed = 0 # rad/s

    
    # linear, angular
    def speed_cur(self):
        return (self.r_wheel_speed-self.l_wheel_speed)/self.width, (self.r_wheel_speed + self.l_wheel_speed)/2

    # linear, angular
    def speed_out(self, linear_ideal, angular_ideal):

        # ideal to wheels
        l_wheel_speed_ideal = (2*linear_ideal - angular_ideal*self.width)/2
        r_wheel_speed_ideal = (2*linear_ideal + angular_ideal*self.width)/2

        # wheels control
        self.l_wheel_speed = self.l_wheel_pid(l_wheel_speed_ideal-self.l_wheel_speed)
        self.r_wheel_speed = self.r_wheel_pid(r_wheel_speed_ideal-self.r_wheel_speed)

        # wheels to output
        return linear_ideal, angular_ideal # remove this later
        return self.speed_cur()
    