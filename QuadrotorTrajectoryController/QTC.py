import numpy as np

class State:
    def __init__(self):
        self.position = np.zeros((3,1))
        self.velocity = np.zeros((3,1))
        
class UserCode:
    def __init__(self):
        # TODO: tune gains
    
        # xy control gains
        Kp_xy = 1.0 # xy proportional
        Kd_xy = 0.2 # xy differential
        
        # height control gains
        Kp_z  = 0.45 # z proportional
        Kd_z  = 0.15 # z differential
        
        self.Kp = np.array([[Kp_xy, Kp_xy, Kp_z]]).T
        self.Kd = np.array([[Kd_xy, Kd_xy, Kd_z]]).T
        self.Kp_xy = Kp_xy
        self.Kd_xy = Kd_xy
        self.Kp_z = Kp_z
        self.Kd_z = Kd_z
        
    def compute_control_command(self, t, dt, state, state_desired):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param state: State - current quadrotor position and velocity computed from noisy measurements
        :param state_desired: State - desired quadrotor position and velocity
        :return - xyz velocity control signal represented as 3x1 numpy array
        '''
        # plot current state and desired setpoint
        self.plot(state.position, state_desired.position)
        
        # TODO: implement PID controller computing u from state and state_desired
        u = np.zeros((3,1))
        error = state_desired.position - state.position
        d = state_desired.velocity - state.velocity
        u = self.Kp_xy * error + self.Kd * d
        self.vel_error = error        
        return u
        
    def plot(self, position, position_desired):
        from plot import plot
        plot("x", position[0])
        plot("x_des", position_desired[0])
        plot("y", position[1])
        plot("y_des", position_desired[1])
        plot("z", position[2])
        plot("z_des", position_desired[2])
        
