#!/usr/bin/env python

#from IK import Kuka_IK
from argparse import ArgumentParser
import subprocess
from Collections import OrderedDict
import ast

def getPose(ref_link, link):
    result = subprocess.run('timeout 1 rosrun tf tf_echo {0} {1} .1'.format(ref_link, link),
        stdout=subprocess.PIPE).stdout.decode('utf-8')

    position = subprocess.run(["grep 'Translation:'", result],
        stdout=subprocess.PIPE).stdout.decode('utf-8')

    rotation = subprocess.run(["grep 'in RPY (radian)'", result],
        stdout=subprocess.PIPE).stdout.decode('utf-8')

    position = ast.literal_eval(position)
    rotation = ast.literal_eval(rotation)

    return [position, rotation]

def printPose(link):
    print('{0} pose:')
    print('Position: {0}'.format(link['position']))
    print('Rotation: {0}'.format(link['rotation']))

#ik = Kuka_IK()

joints = ['link 1', 'link 2', 'link 3', 'link 4',
        'link 5', 'link 6', 'gripper_link']

pose = OrderedDict(joints)

for j in pose:
    pose[j] = {'position': None, 'rotation': None}

    pose[j]['position'], pose[j]['rotation'] = getPose('base_link', 'gripper_link')

    printPose(pose[j])

#[px, py, pz] = position
#[roll, pitch, yaw] = rotation_RPY

#joint_angles = ik.calculateJointAngles(px, py, pz, roll, pitch, yaw)

#for i, q in enumerate(joint_angles):
#    print()