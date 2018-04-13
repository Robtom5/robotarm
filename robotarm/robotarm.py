from .robotparts import (BaseFrame, 
    RevoluteJoint, PrismaticJoint, EndEffector)
from typing import Dict, Tuple, List

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RobotArm(object):
    def __init__(self, coords, rotation=(0,0)):
        self.joints = [BaseFrame(loc=coords, rot=rotation)]

    def add_revolute(self, d=0, theta=0, a=0, alpha=0) -> None:
        '''Adds a revolute joint to the end of the robot arm'''
        self.joints.append(
            RevoluteJoint(d, theta, a, alpha, self.joints[-1]))

    def add_prismatic(self, d=0, theta=0, a=0, alpha=0) -> None:
        '''Adds a prismatic joint to the end of the robot arm'''
        self.joints.append(
            PrismaticJoint(d, theta, a, alpha, self.joints[-1]))

    def add_end_effector(self, d=0, theta=0, a=0, alpha=0) -> None:
        self.joints.append(
            EndEffector(d, theta, a, alpha, self.joints[-1]))

    def forward_kinematics(self, targetJoint=-1):
        '''
        Calculates the transformation matrix for the arm up to the 
        specified joint. If no joint is chosen then evaluates to the 
        end effector
        '''
        if targetJoint <= len(self.joints):
            return self.joints[targetJoint].evaluate_forwards()
        else:
            raise IndexError("Must provide valid joint number")

    def set_arm_values(self, new_angles: list) -> bool:
        '''
        Updates all the joints in the arms to the provided angles.
        A value of None can be set to leave a joint as it currently is.
        '''
        if len(new_angles) != len(self.joints[1:]):
            raise ValueError("Values must be provided for all"+
                "joints, use None for no change")
            return False

        for arm, angle in zip(self.joints[1:], new_angles):
            if angle is not None:
                arm.set_joint_value(angle)

        return True

    def change_arm_values(self, new_angles: list) -> bool:
        '''
        Adjusts the joints in the arm by the list of provided values.
        This allows for a relative change instead of absolute.
        '''
        if len(new_angles) != len(self.joints[1:]):
            raise ValueError("Values must be provided for all"+
                "joints, use None for no change")
            return False

        for arm, angle in zip(self.joints[1:], new_angles):
            if angle is not None:
                arm.change_joint_value(angle)

        return True

    @property
    def dh_matrices(self):
        return [joint.evaluate_forwards() for joint in self.joints]

    @property
    def jointLocations(self) -> List[tuple]:
        '''Returns a list of tuples of each joints location in 3d space'''
        # NB: currently inefficient for many joints as does not 
        # cache previous transformation matrices
        return [joint.jointCoordinates for joint in self.joints]
