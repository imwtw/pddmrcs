class dc_motor:
    def __init__(self, *, Ra, La, K, B, J_internal, dt):
        self.Ra = Ra               
        self.La = La               
        self.K = K                 
        self.B = B                          
        self.J_internal = J_internal        
        self.J_external = 0
        self.J = self.J_internal + self.J_external
        self.dt = dt
        self.I = 0.
        self.T = 0.
        self.voltage = 0.
        self.ang_acc = 0.
        self.ang_vel = 0.  
        self.Tf = .14

    def update_J(self, *, J_ext):
        self.J_external = J_ext
        self.J = self.J_internal + self.J_external

    def update(self, *, voltage):
        self.voltage = voltage
        dI = ((voltage - self.K * self.ang_vel - self.Ra * self.I) / self.La) * self.dt 
        self.I = self.I + dI
        friction_direction = 0
        if (self.ang_vel > 0):  
            friction_direction = 1
        elif (self.ang_vel < 0):
            friction_direction = -1
        self.T = self.K * self.I  - self.B * self.ang_vel   
        if (abs(self.T) < self.Tf):  
            self.T = 0.
        else:
            self.T -= friction_direction * self.Tf
        # self.T = self.K * self.I  - self.B * self.ang_vel  - self.Tf * friction_direction
        self.ang_acc = (self.T) / self.J                               
        self.ang_vel = self.ang_vel + self.ang_acc * self.dt
        

    def get_speed(self) -> float:
        return self.ang_vel
    
    def get_current(self) -> float:
        return self.I
    
    def get_torque(self) -> float:
        return self.T

    def get_voltage(self) -> float:
        return self.voltage