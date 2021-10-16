#!/usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from markers import *
from lab4functions import *


class Joystick(object):

    def __init__(self):
        self.axes = 6*[0.,]
        self.buttons = 6*[0.,]
        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, msg):
        self.axes = msg.axes
        self.buttons = msg.buttons


rospy.init_node("joystick_ikine")

joystick = Joystick()

bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['GREEN'])
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Desired position
xd = np.array([0.94, 0.125, 0.249])
# Initial configuration
q = np.array([0.0, -1.57 , 0.0 , -1.57 , 0.0 , 0.0])


# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
# Objeto (mensaje) de tipo JointState
jstate = JointState()

while not rospy.is_shutdown():
    # Wait for the next iteratio
    if joystick.axes[0]>0:

        xd[0]=xd[0]+0.05
        
    elif joystick.axes[0]<0:
        xd[0]=xd[0]-0.05

    elif joystick.axes[1]>0:

        xd[2]=xd[2]+0.05
        
    elif joystick.axes[1]<0:
        xd[2]=xd[2]-0.05   

    elif joystick.axes[3]>0:

        xd[1]=xd[1]+0.05
        
    elif joystick.axes[3]<0:
        xd[1]=xd[1]-0.05       

    # Inverse kinematics
    q_aux = ikine_ur5(xd, q)
    T = fkine_ur5(q_aux)
    # Red marker shows the achieved position
    bmarker.xyz(T[0:3,3])
    # Green marker shows the desired position
    bmarker_des.xyz(xd)

    
    
    bmarker.publish()
    bmarker_des.publish()
  #  for x in range(len(q_aux)):
   #     if q_aux[x]>1.57 and q_aux[x]<-1.57:
    #        q_aux=q
            
            

    
    if joystick.axes[0]!=0 or joystick.axes[1]!=0:

        q=q_aux
    
    # Asignar valores al mensaje
    jstate.header.stamp = rospy.Time.now()
    jstate.name = jnames
    # Add the head joint value (with value 0) to the joints
    jstate.position = q
    pub.publish(jstate)
    print("axes:")
    print(joystick.axes)
    print("articulaciones:")
    print(q/1000)
    rate.sleep()