#!/usr/bin/env python

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from IK import Kuka_IK

class Pose_Error(object):
    #class to record position and orientation error
    #for each IK iteration
    def __init__(self):
        pass

class handle_IK(object):
    def __init__(self, Kuka_IK_object):
        ik = Kuka_IK_object

    def handle_calculate_IK(self, req):
        rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
        if len(req.poses) < 1:
            print "No valid poses received"
            return -1
        else:
            # Initialize service response
            joint_trajectory_list = []
            for x in xrange(0, len(req.poses)):
                # IK code starts here
                joint_trajectory_point = JointTrajectoryPoint()

               # px,py,pz = end-effector position
               # roll, pitch, yaw = end-effector orientation (in gazebo frame)
                px = req.poses[x].position.x
                py = req.poses[x].position.y
                pz = req.poses[x].position.z

                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

                # Populate response for the IK request
                # In the next line replace theta1,theta2...,theta6 by your joint angle variables
                joint_trajectory_point.positions = ik.calculateJointAngles(px, py, pz, roll, pitch, yaw)
                joint_trajectory_list.append(joint_trajectory_point)

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
            return CalculateIKResponse(joint_trajectory_list)

    def IK_server(self):
        # initialize node and declare calculate_ik service
        rospy.init_node('IK_server')
        s = rospy.Service('calculate_ik', CalculateIK, self.handle_calculate_IK)
        print "Ready to receive an IK request"
        rospy.spin()

if __name__ == "__main__":
    #instantiate Kuka_IK object
    ik = Kuka_IK()

    #instantiate handle_IK object
    h = handle_IK(ik)

    #run the IK server
    h.IK_server()
