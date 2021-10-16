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

class Joystick(object):

    def __init__(self):
        self.axes = 6*[0.,]
        self.buttons = 6*[0.,]
        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, msg):
        self.axes = msg.axes
        self.buttons = msg.buttons

if __name__ == '__main__':
    
    rospy.init_node("test1", disable_signals=True)

    robot_client = actionlib.SimpleActionClient('follow_joint_trajectory',
                                                FollowJointTrajectoryAction)

    print "Waiting for server..."
    robot_client.wait_for_server()
    print "Connected to server"
    joystick = Joystick()

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
    
    # Desired position
    xd = np.array([0.94, 0.125, 0.249])
    # Initial configuration
    l=0
    rate = rospy.Rate(100)
    equ_esfera=xd[0]*xd[0]+xd[1]*xd[1]+(xd[2]-0.08)*(xd[2]-0.08)
    while not rospy.is_shutdown():
        #robot_client.cancel_goal()

        # Modification of the motion
               
        if joystick.axes[0]>0 :

            xd[0]=xd[0]+joystick.axes[0]/1000
            print('1')
                       
        

        elif joystick.axes[1]>0:

            xd[2]=xd[2]+joystick.axes[1]/1000
            print('2')
            
        elif joystick.axes[1]<0:
            xd[2]=xd[2]-joystick.axes[1]/1000 
            print('5')  

        elif joystick.axes[3]>0:

            xd[1]=xd[1]+joystick.axes[3]/1000
            print('3')
            
        elif joystick.axes[3]<0:
            xd[1]=xd[1]-joystick.axes[3]/1000  
            print('6')    

        # Inverse kinematics
        equ_esfera=xd[0]*xd[0]+xd[1]*xd[1]+(xd[2]+0.08)*(xd[2]+0.08)
        if equ_esfera>=0.96:
            xd = np.array([xd[0], xd[1], xd[2]])
            
        else:
            xd = np.array([0.94, 0.125, 0.249])



        Q0= ikine_ur5(xd, Q0)
         
        

        #if joystick.axes[0]!=0 or joystick.axes[1]!=0:
         # Q0[0] = Q0[0]-0.05
                    
        # if(Q0[1] >= np.pi/2):
        #     Q0[1] = np.pi/2
        
        # if(Q0[2] >= np.pi/2):
        #     Q0[2] = np.pi/2
        
        # if(Q0[3] >= np.pi/2):
        #     Q0[3] = np.pi/2
        # #
        

        
        g.trajectory.points = [ JointTrajectoryPoint(positions=Q0, velocities=[0]*6,
                                                        time_from_start=rospy.Duration(0.008))]
        robot_client.send_goal(g)
        robot_client.wait_for_result()
        print(robot_client.get_state())
    
        rate.sleep()

    robot_client.cancel_goal()

    
    