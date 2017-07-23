#!/usr/bin/env python

"""
Class for calculating inverse kinematics of the Kuka kr210 arm
"""

from mpmath import *
from sympy import *
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
                  self.a4:      0, self.alpha4: -pi/2, self.d5:             0, self.q5: self.q5,
                  self.a5:      0, self.alpha5:  pi/2, self.d6:             0, self.q6: self.q6,
                  self.a6:      0, self.alpha6:     0, self.d7: 0.2305+0.0725, self.q7: 0}

        # Create individual transformation matrices
        self.T0_1 = self.body_fixed_transformation(self.s,1)
        self.T1_2 = self.body_fixed_transformation(self.s,2)
        self.T2_3 = self.body_fixed_transformation(self.s,3)
        self.T3_4 = self.body_fixed_transformation(self.s,4)
        self.T4_5 = self.body_fixed_transformation(self.s,5)
        self.T5_6 = self.body_fixed_transformation(self.s,6)
        self.T6_G = self.body_fixed_transformation(self.s,7)

        self.T0_2 = self.T0_1*self.T1_2
        self.T0_4 = self.T0_2*self.T2_3*self.T3_4
        self.T0_G = self.T0_4*self.T4_5*self.T5_6*self.T6_G
        self.T4_G = self.T0_4.T*self.T0_G

        #Correction for orientation difference between UDRF Gripper location and
        #modified DH parameter conventions
        #rotate 180 degrees about z, then -90 degrees about y
        self.R_corr = self.rot_z(180)*self.rot_y(-90)

        ##Define constants used in inverse kinematics
        self.beta = pi/2 + atan2(self.s[self.a3], self.s[self.d4])
        self.l3 = (self.s[self.a3]+ self.s[self.d4])**0.5
        self.a2_const = self.s[self.a2]
        self.a3_const = self.s[self.a3]
        self.d4_const = self.s[self.d4]
        self.d7_const = self.s[self.d7]

        self.consts = {'beta': self.beta, 'l3': self.l3, 'a2': self.a2_const, 'a3': self.a3_const, 'd4': self.d4_const}

        #initialize volatile attributes as None
        self.P = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.wc = None
        self.q1_res = None
        self.q2_res = None
        self.q3_res = None
        self.q4_res = None
        self.q5_res = None
        self.q6_res = None
        self.r24 = None

    # Define Modified DH Transformation matrix
    def body_fixed_transformation(self, s, i):
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

        return transform.subs(s)

    def Tx(self, q):
        #apply rotation about x
        q *= pi/180
        T_x = Matrix([[ 1,              0,        0, 0],
                      [ 0,        cos(q),   -sin(q), 0],
                      [ 0,        sin(q),    cos(q), 0],
                      [ 0,             0,         0, 1]])
        
        return T_x
        
    def Ty(self, q):
        #apply rotation about y 
        q *= pi/180
        T_y = Matrix([[  cos(q),        0,   sin(q), 0],
                      [       0,        1,        0, 0],
                      [ -sin(q),        0,   cos(q), 0],
                      [       0,        0,        0, 1]])
        
        return T_y

    def Tz(self, q): 
        #apply rotation about z
        q *= pi/180
        T_z = Matrix([[ cos(q),   -sin(q),     0, 0],
                      [ sin(q),    cos(q),     0, 0],
                      [      0,         0,     1, 0],
                      [      0,         0,     0, 1]])
        
        return T_z

    def rot_x(self, q):
        #q *= pi/180
        #R_x = Matrix([[ 1,              0,        0],
        #              [ 0,        cos(q), -sin(q)],
        #              [ 0,        sin(q),  cos(q)]])
        
        #return R_x
        return self.Tx(q)[:3,:3]
        
    def rot_y(self, q): 
        #q *= pi/180
        #R_y = Matrix([[ cos(q),        0,  sin(q)],
        #              [       0,        1,        0],
        #              [-sin(q),        0,  cos(q)]])
        
        #return R_y
        return self.Ty(q)[:3,:3]

    def rot_z(self, q): 
        #q *= pi/180
        #R_z = Matrix([[ cos(q), -sin(q),        0],
        #              [ sin(q),  cos(q),        0],
        #              [ 0,              0,        1]])
        
        #return R_z
        return self.Tz(q)[:3,:3]

    def forwardKinematics(joint_angles):
        s = dict(zip([q1,q2,q3,q4,q5,q6],joint_angles))

        joint_positions = OrderedDict()

        for i in ['1','2','3','4','5','6','G']:
            eval('T_0{0} = self.T_0{0}.evalf(subs = s)'.format(i))
            joint_positions[i] = eval('T_0{0}[0:3, 2]')

            print('Joint {0}: {1}'.format(i, joint_positions[i]))

        #return joint_positions

    def return_theta1(self):
        return atan2(self.wc[1],self.wc[0])

    def return_theta2(self):
        #return theta 2 from
        #l3 = const. length from joint 3 coord. sys to joint 4 coord. sys
        #a2 = const. from DH params
        #r24 = 3x1 vector from joint 2 to joint 4

        l3 = self.consts['l3']
        a2 = self.consts['a2']
        r24 = self.r24

        r24z = r24[2]
        r24xy = (r24[0]**2 + r24[1]**2)**0.5
        r24_mag = (r24[0]**2 + r24[1]**2 + r24[2]**2)**0.5

        acos_term = acos((-l3**2 + r24_mag**2 + a2**2)/(2*a2*r24_mag))
        theta2_term1 = pi/2 + atan2(r24z, r24xy)

        #return both possible values of theta2 given acos uncertainty
        theta2 = [theta2_term1 + acos_term, theta2_term1 - acos_term]

        return theta2

    def return_theta3(self):
        #return theta 3 from
        #a2 = const. from DH params
        #r24 = 3x1 vector from joint 2 to joint 4
        #beta = const. defined as pi/2 + tan(a3/d4)

        a2 = self.consts['a2']
        l3 = self.consts['l3']
        beta = self.consts['beta']
        r24 = self.r24

        print(r24)

        r24_mag = (r24[0]**2 + r24[1]**2 + r24[2]**2)**0.5
        phi = acos((-r24_mag**2 + a2**2 + l3**2)/(2*a2*l3))

        #return both possible values of theta3 given acos uncertainty
        theta3 = [pi - beta + phi, pi - beta - phi]

        return theta3

    def return_valid_theta23(self, theta2, theta3):
        #Determine which combination of theta 2 and 3 possibilities
        #results in the correct joint 4 position
        for t2 in theta2:
            for t3 in theta3:
                #check if joint 4 position matches wrist center command
                wx, wy, wz = self.T0_4.evalf(subs = {self.q1: self.q1_res, self.q2: t2, self.q3: t3})
                if (wx, wy, wz) == self.wc:
                    return [t2, t3]

        return [None, None]

    def return_theta5(self):
        #evaluate T_04 for theta4 = 0 and known theta1, theta2, theta3
        T_04 = self.T_04.evalf(subs = {self.q1: self.q1_res,
                                       self.q2: self.q2_res,
                                       self.q3: self.q3_res,
                                       self.q4: 0})

        #unit vector along Z4
        n_04 = T_04[0:3, 2]

        #unit vector along Z6
        n_06 = self.Rrpy[0:3, 2]

        dot_product = n_04.dot(n_06)

        theta5 = acos(dot_product)

        #determine if theta5 should be adjusted to 2*pi - theta5
        M = T_04[0:3,0:3]*(n_06-n_04)

        if M[0,3]< 0:
            #if vector from n_06 to n_04 has negative X component in joint 4 frame
            theta5 = 2*pi - theta5

        return theta5

    def return_theta46(self):
        T_46 = Transpose(self.T_04)*self.T_06
        T_46 = T_46.evalf(subs = {self.q1: self.q1_res,
                                  self.q2: self.q2_res,
                                  self.q3: self.q3_res,
                                  self.q5: self.q5_res,
                                  })

        theta4 = atan2(T_46[1,2], T_46[1,3])
        theta6 = atan2(-T_46[2,1], T_46[2,0])

        return [theta4, theta6]

    def get_r24(self):
        T0_2 = self.T0_2.evalf(subs = {self.q1:self.q1_res})
        r24 = Matrix(self.wc) - T0_2[:3,3]

        return r24

    def getRrpy(self):
        #EE rotation matrix
        Rrpy = self.rot_x(self.roll)*self.rot_y(self.pitch)*self.rot_z(self.yaw)

        #apply correction to orientation to account for coord misalignment
        Rrpy = Rrpy*self.R_corr

        return Rrpy

    def getWristCenter(self):
        #wrist center wc = [[wx], [wy], [wz]] in base coords
        wrist = self.P - self.Rrpy*Matrix(3,1,[0,0,self.d7_const])

        wc = (wrist[0], wrist[1], wrist[2])

        return wc

    def calculateJointAngles(self, px, py, pz, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        self.P = Matrix([[px],[py],[pz]])

        # Calculate joint angles using Geometric IK method

        ##Calculate end effector orientation rotation matrix
        self.Rrpy = self.getRrpy()

        ##Calculate wrist center position
        self.wc = self.getWristCenter()

        ##Find theta1 (q1)
        self.q1_res = self.return_theta1()

        ##Calculate r24 (vector from joint 2 to joint 4)
        self.r24 = self.get_r24()

        ##Calculate theta2 and theta3 possibilities
        theta2 = self.return_theta2()
        theta3 = self.return_theta3()

        ##Determine which pair of theta2 and theta3 are correct
        self.q2_res, self.q3_res = self.return_valid_theta23(theta2, theta3)

        ##Calculate theta5
        self.q5_res = self.return_theta5()

        ##Calculate theta4 and theta6
        self.q4_res, self.q6_res = self.return_theta46()

        return [self.q1_res, self.q2_res, self.q3_res, self.q4_res, self.q5_res, self.q6_res]
