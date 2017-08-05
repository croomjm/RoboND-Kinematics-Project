#!/usr/bin/env python

"""
Class for calculating inverse kinematics of the Kuka kr210 arm
"""

from mpmath import *
from sympy import *
import tf
from collections import OrderedDict

class Kuka_IK(object):
    def __init__(self):

        #Define DH param variables
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6, self.q7 = symbols('q1:8') #thetas
        self.d1, self.d2, self.d3, self.d4, self.d5, self.d6, self.d7 = symbols('d1:8')
        self.a0, self.a1, self.a2, self.a3, self.a4, self.a5, self.a6 = symbols('a0:7')
        self.alpha0, self.alpha1, self.alpha2, self.alpha3, self.alpha4, self.alpha5, self.alpha6 = symbols('alpha0:7')

        # Modified DH params
        self.s = {self.a0:      0, self.alpha0:     0, self.d1:          0.75, self.q1: self.q1, 
                  self.a1:   0.35, self.alpha1: -pi/2, self.d2:             0, self.q2: self.q2 - pi/2,
                  self.a2:   1.25, self.alpha2:     0, self.d3:             0, self.q3: self.q3,
                  self.a3: -0.054, self.alpha3: -pi/2, self.d4:           1.5, self.q4: self.q4,
                  self.a4:      0, self.alpha4:  pi/2, self.d5:             0, self.q5: self.q5,
                  self.a5:      0, self.alpha5: -pi/2, self.d6:             0, self.q6: self.q6,
                  self.a6:      0, self.alpha6:     0, self.d7: 0.2305+0.0725, self.q7: 0}

        self.T = self._buildTransforms(self.s)

        #Correction for orientation difference between UDRF Gripper location and
        #modified DH parameter conventions
        #rotate 180 degrees about z, then -90 degrees about y
        self.R_corr = self._rot('Y',-90)*self._rot('Z',180)

        ##Define constants used in inverse kinematics
        self.consts = {'gamma': atan2(-self.s[self.a3],self.s[self.d4]),
                       'l3': (self.s[self.a3]**2+ self.s[self.d4]**2)**0.5,
                       'a2': self.s[self.a2],
                       'a3': self.s[self.a3],
                       'd1': self.s[self.d1],
                       'd4': self.s[self.d4],
                       'd7': self.s[self.d7]}

    # Define Modified DH Transformation matrix
    def _body_fixed_transformation(self, s, i):
        '''
        Take in DH parameters for joints i-1 to i.
        Return transformation matrix from i-1 to i.
        For brevity, theta = theta_i, alpha = alpha_(i-1), a = a_(i-1), d = d_i
        '''

        #set variables using globally-defined symbols and input i
        theta = eval('s[self.q{0}]'.format(i))
        alpha = eval('s[self.alpha{0}]'.format(i-1))
        a = eval('s[self.a{0}]'.format(i-1))
        d = eval('s[self.d{0}]'.format(i))

        transform = Matrix([[            cos(theta),           -sin(theta),            0,              a],
                            [ sin(theta)*cos(alpha), cos(theta)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
                            [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d],
                            [                     0,                     0,            0,              1]])

        return transform

    def _buildTransforms(self, s):
        T = {}
        for i in range(1,8):
            T[(i-1,i)] = self._body_fixed_transformation(s, i)
            if i>1:
                T[(0,i)] = T[(0,i-1)]*T[(i-1,i)]

        T[(3,6)] = T[(3,4)]*T[(4,5)]*T[(5,6)]

        return T

    def _rot(self, axis, q):
        '''
        Return rotation matrix about specified axis
        given rotation angle q (in radians).
        '''

        q *= 180/pi

        if axis == 'X':
            R = [[ 1,      0,       0],
                 [ 0, cos(q), -sin(q)],
                 [ 0, sin(q),  cos(q)]]
        elif axis == 'Y':
            R = [[  cos(q), 0, sin(q)],
                 [       0, 1,      0],
                 [ -sin(q), 0, cos(q)]]
        elif axis == 'Z':
            R = [[ cos(q), -sin(q), 0],
                 [ sin(q),  cos(q), 0],
                 [      0,       0, 1]]
        else:
            raise RuntimeError('{} is not a valid axis. Options are "X", "Y", or "Z".'.format(axis))

        return Matrix(R)

    def forwardKinematics(self, joint_angles):
        '''
        Return joint, wc, and ee positions given joint_angles
        and precalculated transformation matrices between joints
        '''

        s = dict(zip([self.q1, self.q2, self.q3, self.q4, self.q4, self.q5, self.q6],
              joint_angles))

        joint_positions = OrderedDict()

        for i in range(1,8):
            joint_positions[i] = self.T[(0,i)][:3,3].evalf(subs = s)
            #print('Joint {0}: {1}'.format(i, list(joint_positions[i])))

        return joint_positions

    def calculateIKError(self, wc_target, ee_target, joint_angles):
        '''
        Calculate error between actual end effector and wrist center
        positions versus target positions.
        '''

        joint_positions = self.forwardKinematics(joint_angles)
        wc_actual = joint_positions[4]
        ee_actual = joint_positions[7]

        wc_error = (wc_target - wc_actual).norm()
        ee_error = (ee_target - ee_actual).norm()

        return [wc_actual, wc_error, ee_actual, ee_error]

    def _returnTheta1(self, wc):
        return atan2(wc[1],wc[0])

    def _returnTheta23(self, consts, r24, default_orientation = True):
        #return theta 2 and theta 3 using law of cosines with sides
        ##r24 (vector from joint 2 to joint 4 coord frame)
        ##a2 (length of link 2)
        ##l3 (legnth of line from joint 3 to joint 4 coord frame)
        ##angle_a between r24 and a2
        ##angle_b between a2 and l3
        ##gamma angle of declination of link 3
        ##angle r24 (angle of inclination of r24 from XY plane)

        gamma = consts['gamma']
        l3 = consts['l3']
        a2 = consts['a2']

        r24z = r24[2]
        r24xy = (r24[0]**2 + r24[1]**2)**0.5
        angle_r24 = atan2(r24z, r24xy)
        r24_mag = (r24[0]**2 + r24[1]**2 + r24[2]**2)**0.5

        angle_a = acos((-l3**2 + a2**2 + r24_mag**2)/(2*a2*r24_mag))
        angle_b = acos((-r24_mag**2 + a2**2 + l3**2)/(2*a2*l3))

        if default_orientation:
            #if joint 4 is below vector from joint 2 to joint 3
            theta2 = pi/2 - angle_a - angle_r24
            theta3 = pi/2 - gamma - angle_b
        # else:
        #     theta2 = pi/2 - angle_r24 + a
        #     theta3 = pi/2 - 

        return [theta2, theta3]

    def _returnTheta456(self, theta1, theta2, theta3, Rrpy):
        '''
        Calculate theta 4, theta5, theta6 based on relations
        between elements in the rotation matrix from frame 3 to 6.
        '''
        #Use rotation matrix from joint 3 to 6 to calculate
        #theta 4, theta5, theta6
        R_03 = self.T[(0,3)][:3,:3].evalf(subs = {self.q1: theta1,
                                                  self.q2: theta2,
                                                  self.q3: theta3})

        R_36 = R_03.T*Rrpy
        print('R_36:')
        pprint(R_36.evalf())

        theta4 = atan2(R_36[2,2], -R_36[0,2])
        theta5 = atan2((R_36[0,2]**2 + R_36[2,2]**2)**0.5, R_36[1,2])
        theta6 = atan2(-R_36[1,1], R_36[1,0])

        return [theta4, theta5, theta6]

    def _return_r24(self, wc, q1):
        r02 = self.T[(0,2)][:3,3].evalf(subs = {self.q1: q1})
        r24 = wc - r02

        return r24.evalf()

    def _returnRrpy(self, quaternion):
        #EE rotation matrix
        Trpy = tf.transformations.quaternion_matrix(quaternion)
        Rrpy = Matrix(Trpy[:3,:3])

        #apply correction to orientation to account for coord misalignment
        Rrpy = Rrpy*self.R_corr

        return Rrpy

    def _returnWristCenter(self, consts, EE_target, Rrpy):
        #wrist center wc = [[wx], [wy], [wz]] in base coords
        wc_target = EE_target - Rrpy*Matrix([0,0,consts['d7']])
        wc_target = wc_target.evalf()

        return wc_target

    def calculateJointAngles(self, px, py, pz, quaternion):
        #Calculate end effector orientation rotation matrix
        #from EE quaternion
        Rrpy = self._returnRrpy(quaternion)

        #Calculate wrist center and end effector target positions
        ee_target = Matrix([px, py, pz])
        wc_target = self._returnWristCenter(self.consts, ee_target, Rrpy)
        true_wc_target = [1.89451,-1.44302,1.69366]
        print('Calculated wrist center target: {0}'.format(list(wc_target)))
        print('Correct wrist center target: {0}'.format(true_wc_target))

        #override for debugging purposes
        wc_target = Matrix(true_wc_target)

        theta1 = self._returnTheta1(wc_target)
        print('Theta1: {0}'.format(theta1))

        #Calculate r24 (vector from joint 2 to joint 4)
        r24 = self._return_r24(wc_target, theta1)

        theta2, theta3 = self._returnTheta23(self.consts, r24)
        print('Theta2: {0}'.format(theta2.evalf()))
        print('Theta3: {0}'.format(theta3.evalf()))

        theta4, theta5, theta6 = self._returnTheta456(theta1, theta2, theta3, Rrpy)
        print('Theta4: {0}'.format(theta4.evalf()))
        print('Theta5: {0}'.format(theta5.evalf()))
        print('Theta6: {0}'.format(theta6.evalf()))

        joint_angles = [theta1, theta2, theta3, theta4, theta5, theta6]

        [wc_actual,
         wc_error,
         ee_actual,
         ee_error] = self.calculateIKError(wc_target, ee_target, joint_angles)

        print('Wrist Center Error: {0:0.4f} mm'.format(wc_error*1000))
        print('End Effector Error: {0:0.4f} mm'.format(ee_error*1000))

        return [joint_angles, wc_actual, wc_error, ee_actual, ee_error]
