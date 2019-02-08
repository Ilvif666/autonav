import numpy as np

class Pose3D:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation
        
    def inv(self):
        '''
        Inversion of this Pose3D object
        
        :return inverse of self
        '''
        # TODO: implement inversion
        inv_rotation = np.transpose(self.rotation)
        inv_translation = np.dot(-inv_rotation,self.translation)
        '''
        Tqw*Tmq=Tmw  => Tqw=Tmw*(Tmq)^-1
        so we need T^-1, this is it : [[R^t, -R^T*t],
                                        [0,     1]]
        where R^T is transposed rotation matrix 3x3 and t  is translation vector 3x1
        '''
        return Pose3D(inv_rotation, inv_translation)
    
    def __mul__(self, other):
        '''
        Multiplication of two Pose3D objects, e.g.:
            a = Pose3D(...) # = self
            b = Pose3D(...) # = other
            c = a * b       # = return value
        
        :param other: Pose3D right hand side
        :return product of self and other
        '''
        # TODO: implement multiplication
        '''
        [[R1, t1], *[[R2, t2],  =[[R1*R2, R1*t2+t1]
        [0,  1]]     [0, 1]]     [0,         1]]
        '''
        
        self.translation = self.translation + np.dot(self.rotation,other.translation)
        self.rotation = np.dot(self.rotation, other.rotation)
        return Pose3D(self.rotation, self.translation)
    
    def __str__(self):
        return "rotation:\n" + str(self.rotation) + "\ntranslation:\n" + str(self.translation.transpose())

def compute_quadrotor_pose(global_marker_pose, observed_marker_pose):
    '''
    :param global_marker_pose: Pose3D 
    :param observed_marker_pose: Pose3D
    
    :return global quadrotor pose computed from global_marker_pose and observed_marker_pose
    '''
    # TODO: implement global quadrotor pose computation
    global_quadrotor_pose = global_marker_pose*observed_marker_pose.inv()
    return global_quadrotor_pose
