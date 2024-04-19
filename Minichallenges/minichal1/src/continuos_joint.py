#!/usr/bin/env python3 

import rospy 
import numpy as np 
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState 

# Declare the output Messages 

global wl , wr

wl = 0 
wr = 0

global contJoints

contJoints = JointState() 

# Declare the output Messages 

def init_joints(): 
    contJoints.header.frame_id = "base_link" 
    contJoints.header.stamp = rospy.Time.now() 
    contJoints.name.extend(["wl_joint", "wr_joint"]) 
    contJoints.position.extend([0.0, 0.0]) 
    contJoints.velocity.extend([0.0, 0.0]) 
    contJoints.effort.extend([0.0, 0.0]) 

def wr_cb(msg):
    global wr
    wr = msg.data

def wl_cb(msg):
    global wl
    wl = msg.data

#wrap to pi function 
def wrap_to_Pi(theta): 
    result = np.fmod((theta + np.pi),(2 * np.pi)) 
    if(result < 0): 
        result += 2 * np.pi 
    return result - np.pi 


def stop(): 
    #Setup the stop message (can be the same as the control message) 
    print("Stopping") 
 

if __name__=='__main__': 
    #Initialise and Setup node 
    rospy.init_node("Puzzlebot_Pose_Estimator") 
    # Configure the Node 
    loop_rate = rospy.Rate(rospy.get_param("/rate",50)) 
    rospy.on_shutdown(stop) 
    #Init joints 
    init_joints() 
    #Setup Transform Broadcasters 
    rospy.Subscriber("wr" , Float32 , wr_cb)
    rospy.Subscriber("wl" , Float32 , wl_cb)
    joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1) 
    print("The Estimator is Running")

    try: 
    #Run the node 
        while not rospy.is_shutdown():  
#Change state of the joint 
            t = rospy.Time.now().to_sec() 
            contJoints.header.stamp = rospy.Time.now() 
            contJoints.position[0] = wrap_to_Pi(wl * t) 
            contJoints.position[0] += wr / t

            contJoints.position[1] = wrap_to_Pi(wr * t)  
            joint_pub.publish(contJoints) 
            loop_rate.sleep() 

    except rospy.ROSInterruptException: 
        pass