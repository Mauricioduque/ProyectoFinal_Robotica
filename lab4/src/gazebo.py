#!/usr/bin/env python

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import numpy as np
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import Joy
from lab4functions import *


if __name__ == '__main__':
    
    rospy.init_node("test1", disable_signals=True)

    robot_client = actionlib.SimpleActionClient('follow_joint_trajectory',
                                                FollowJointTrajectoryAction)

    print "Waiting for server..."
    robot_client.wait_for_server()
    print "Connected to server"

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    Q0 = [0.0, -1.0, 1.7, -2.2, -1.6, 0.0]

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = joint_names

    # Initial position
    g.trajectory.points = [ JointTrajectoryPoint(positions=Q0, velocities=[0]*6,
                                                 time_from_start=rospy.Duration(2.0))]
    robot_client.send_goal(g)
    robot_client.wait_for_result()
    rospy.sleep(1)

#############
    xd = [0.94, 0.125, 0.249]
    # Initial configuration
    l=0

#########
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        robot_client.cancel_goal()

        # Modification of the motion
        if l<20:
		xd[0]=xd[0]+0.05
		l+=1
	
	Q0= ikine_ur5(xd, Q0)
	if(Q0[0] >= np.pi):
		Q0[0] = np.pi
	elif(Q0[0] <= -np.pi):
		Q0[0] = -np.pi
	if(Q0[1] >= np.pi):
		Q0[1] = np.pi
	elif(Q0[1] <= -np.pi):
		Q0[1] = -np.pi
	if(Q0[2] >= np.pi):
		Q0[2] = np.pi
	elif(Q0[2] <= -np.pi):
		Q0[2] = -np.pi
	if(Q0[3] >= 1.62):
		Q0[3] = 1.62
	elif(Q0[3] <= -4.279):
		Q0[3] = -4.279
	if(Q0[4] >= np.pi):
		Q0[4] = np.pi
	elif(Q0[4] <= -np.pi):
		Q0[4] = -np.pi
	if(Q0[5] >= np.pi):
		Q0[5] = np.pi
	elif(Q0[5] <= -np.pi):
		Q0[5] = -np.pi
	print(xd)
	print(Q0)
        g.trajectory.points = [ JointTrajectoryPoint(positions=Q0, velocities=[0]*6,
                                                     time_from_start=rospy.Duration(0.008))]
        robot_client.send_goal(g)
        robot_client.wait_for_result()

        rate.sleep()

    robot_client.cancel_goal()

    
    
