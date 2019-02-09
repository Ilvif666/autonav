class UserCode:
    def __init__(self):
        # TODO: tune gains
        self.Kp = 1
        self.Kd = 1.9
        self.error = 0   
    def compute_control_command(self, t, dt, x_measured, x_desired):
        '''
        :param t: time since simulation start
        :param dt: time since last call to compute_control_command
        :param x_measured: measured position (scalar)
        :param x_desired: desired position (scalar)
        :return - control command u
        '''
        # TODO: implement PD controller
        error = x_desired - x_measured
        de = error - self.error
        u = self.Kp * error + self.Kd * de/dt
        self.error = error        
        return u
