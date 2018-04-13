import numpy as np
from math import cos, sin, sqrt
import math

class Joint(object):
    def __init__(self, d=0, theta=0, a=0, alpha=0, prevLink=None, **kwargs):
        super().__init__(**kwargs)
        self.d = d
        self.theta = theta
        self.a = a
        self.alpha = alpha
        if not isinstance(prevLink, Joint):
            raise ValueError("Previous link must be a joint")
        self.prevLink = prevLink
        self.prevLink.addLink(self)
        self.nextLink = None

    def set_joint_value(self, newValue):
        setattr(self, self.variable_property, newValue)

    def change_joint_value(self, newValue):
        current = getattr(self, self.variable_property)
        setattr(self, self.variable_property, current+newValue)

    def addLink(self, newLink):
        self.nextLink = newLink

    def evaluate_inverse(self):
        if self.nextLink is None:
            return self.matrix_dh_inv
        else:
            return self.matrix_dh_inv @ self.nextLink.evaluate_inverse()

    def evaluate_forwards(self):
        if self.prevLink is None:
            return self.matrix_dh
        else:
            return self.prevLink.evaluate_forwards() @ self.matrix_dh

    @property
    def matrix_dh(self):
        th = self.theta
        al = self.alpha
        d = self.d
        a = self.a
        return self.make_dh_matrix(a, al, d, th)

    @property
    def matrix_dh_inv(self):
        RT = self.matrixRotation.transpose()
        return np.vstack([np.hstack([
            RT, -RT @ self.matrixTranslation]),
            [0,0,0,1]])

    @property
    def matrixRotation(self):
        return self.matrix_dh[0:3,0:3]

    @property
    def matrixTranslation(self):
        return np.reshape(self.matrix_dh[0:3, 3], (3,1))

    @property
    def jointCoordinates(self):
        return tuple(self.evaluate_forwards()[0:3,3])

    @property
    def link_length(self):
        return sqrt(self.a**2 + self.d**2)

    @staticmethod
    def make_dh_matrix(a, al, d, th):
        return np.array([
            [cos(th),   -sin(th)*cos(al),   sin(th)*sin(al),    a*cos(th)],
            [sin(th),   cos(th)*cos(al),    -cos(th)*sin(al),   a*sin(th)],
            [0,         sin(al),            cos(al),            d],
            [0,         0,                  0,                  1]
            ])

class BaseFrame(Joint):
    variable_property = None

    def __init__(self, loc=(0,0,0), rot=(0,0), *_):
        '''
        Creates a base frame to attach other arm
        parts to at the coordinates specified by loc,
        and a rotation specified by rot.
        '''
        self.d = 0
        self.theta = rot[0]
        self.a = 0
        self.alpha = rot[1]
        self.prevLink = None
        self.nextLink = None
        self.loc = loc

    @property
    def matrix_dh(self):  
        al = self.alpha
        th = self.theta
        return np.array([
            [cos(th),   -sin(th)*cos(al),   sin(th)*sin(al),  self.loc[0]],
            [sin(th),   cos(th)*cos(al),    -cos(th)*sin(al), self.loc[1]],
            [0,         sin(al),            cos(al),          self.loc[2]],
            [0, 0, 0, 1]
            ])

    def set_joint_value(self, _):
        pass

    def change_joint_value(self, _):
        pass

class EndEffector(Joint):
    variable_property = None

    def addLink(self, _):
        raise TypeError("Can't add new link to end efector")

    def set_joint_value(self, _):
        pass

    def change_joint_value(self, _):
        pass

class RevoluteJoint(Joint):
    variable_property = 'theta'

    
class PrismaticJoint(Joint):
    variable_property = 'd'

