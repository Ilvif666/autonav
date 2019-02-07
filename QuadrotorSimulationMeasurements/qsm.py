'''
The navdata contains the following values:

rotX - roll angle in radians (rotation around x axis)
rotY - pitch angle in radians (rotation around y axis)
rotZ - yaw angle in radians (rotation around z axis)
vx - velocity along the x axis
vy - velocity along the y axis
'''
from plot import plot

class UserCode:
    def __init__(self):
        # initialize data you want to store in this object between calls to the measurement_callback() method
        self.last_roll = 0
        self.last_pitch = 0
        self.last_yaw = 0
        self.last_yaw_speed = 0
   
    def measurement_callback(self, t, dt, navdata):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param navdata: measurements of the quadrotor
        '''
        # add your plot commands here
        plot("vx", navdata.vx)
        plot("vy", navdata.vy)
        plot("roll angle", navdata.rotX)
        #roll
        if abs(navdata.rotX) > self.last_roll:
            self.last_roll = abs(navdata.rotX)
            print "roll" + str(self.last_roll)
        #pitch
        if abs(navdata.rotY) > self.last_pitch:
            self.last_pitch = abs(navdata.rotY)
            print "pitch" + str(self.last_pitch)
        #yaw speed
        current_yaw_speed = ((self.last_yaw)-(navdata.rotZ))/dt
        if current_yaw_speed > self.last_yaw_speed:
            self.last_yaw_speed = current_yaw_speed
            print "yaw speed" + str(self.last_yaw_speed)
        self.last_yaw = navdata.rotZ