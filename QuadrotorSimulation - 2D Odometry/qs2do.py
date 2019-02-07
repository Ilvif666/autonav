'''
The navdata contains the following values:

rotX - roll angle in radians (rotation around x axis)
rotY - pitch angle in radians (rotation around y axis)
rotZ - yaw angle in radians (rotation around z axis)
vx - velocity along the x axis
vy - velocity along the y axis
'''
import numpy as np
from plot import plot_trajectory
from math import sin,cos

class UserCode:
    def __init__(self):
        self.position = np.array([[0], [0]])
        
    def measurement_callback(self, t, dt, navdata):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param navdata: measurements of the quadrotor
        '''
        local_velocity = np.array([[navdata.vx], [navdata.vy]])
        prev_position = self.position
        angle = navdata.rotZ
        rotation_to_global = np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])
        velocity = np.array([[navdata.vx], [navdata.vy]])
        self.position += dt*np.dot(rotation_to_global,velocity)
        # TODO: update self.position by integrating measurements contained in navdata
        plot_trajectory("odometry", self.position)
        