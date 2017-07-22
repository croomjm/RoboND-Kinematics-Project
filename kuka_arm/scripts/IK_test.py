#!/usr/bin/env python

#from IK import Kuka_IK
from argparse import ArgumentParser
import subprocess
from collections import OrderedDict
import ast

def getPose(ref_link, link):
    print('Getting pose for base = {0} and link = {1}'.format(ref_link, link))    
    cmd = 'timeout .5 rosrun tf tf_echo {0} {1} .1'.format(ref_link, link)
    #print('Running command: {0}'.format(cmd))
    position = subprocess.run(cmd + " | grep 'Translation:' | sed -n -e 's/^.*: //p'",
        stdout=subprocess.PIPE, shell = True).stdout.decode('utf-8')
    #print('Postion: ', position)

    rotation = subprocess.run(cmd + " | grep 'in RPY (radian)' | sed -n -e 's/^.*(radian) //p'",
        stdout=subprocess.PIPE, shell = True).stdout.decode('utf-8')
    #print('Rotation: ', rotation)

    position = ast.literal_eval(position)
    rotation = ast.literal_eval(rotation)
    #print('Pose calculation complete.')

    return [position, rotation]

def printPose(pose,link):
    print('{0} pose:'.format(link))
    print('Position: {0}'.format(pose[link]['position']))
    print('Rotation: {0}'.format(pose[link]['rotation']))

ik = Kuka_IK()

joints = ['link_1', 'link_2', 'link_3', 'link_4',
        'link_5', 'link_6', 'gripper_link']
vals = [None]*len(joints)

pose = OrderedDict(zip(joints,vals))

for j in pose:
    pose[j] = {'position': None, 'rotation': None}

    pose[j]['position'], pose[j]['rotation'] = getPose('base_link', j)

    printPose(pose[j])

[px, py, pz] = position
[roll, pitch, yaw] = rotation_RPY

joint_angles = ik.calculateJointAngles(px, py, pz, roll, pitch, yaw)

#for i, q in enumerate(joint_angles):
#    print()