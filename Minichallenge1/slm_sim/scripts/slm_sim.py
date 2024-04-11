#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

#Declare Variables to be used
k = 0.01
m = 0.75
l = 0.36 
g = 9.8
Tau = 0.1 

x1 = 0.0
x2 = 0.0
dt = 0.01
a = l/2
j = (4/3)*m*(a**2)
# Setup Variables to be used
# Declare the input Message

# Declare the  process output message
contJoints = JointState() 

def init_joints():

    contJoints.header.frame_id = "base_link"
    contJoints.header.stamp = rospy.Time.now() 

    contJoints.name.extend(["joint1","joint2"]) 

    contJoints.position.extend([0.0, 0.0]) 

    contJoints.velocity.extend([0.0, 0.0]) 

    contJoints.effort.extend([0.0, 0.0])

#Define the callback functions


 


  #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return (np.pi/2*(np.cos(result - np.pi)))- np.pi/2
def pendulum (x1,x2):
    x1 += x2 * dt
    x2_dot = (1/(j+m* a**2)) * (-m*g*a*np.cos(x1)-k*x2 + Tau)
    x2 += x2_dot*dt
    return x2
#Stop Condition 

def stop(): 

    #Setup the stop message (can be the same as the control message) 

    print("Stopping")

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get Parameters   

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    rospy.on_shutdown(stop)

    init_joints() 

    # Setup the Subscribers


    #Setup de publishers

    joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1) 

    print("The SLM sim is Running")
    try:
        #Run the node (YOUR CODE HERE)

        while not rospy.is_shutdown():
        
            #WRITE YOUR CODE HERE
            #WRITE YOUR CODE HERE
            #WRITE YOUR CODE HERE
            t = rospy.Time.now().to_sec() 

            contJoints.header.stamp = rospy.Time.now() 

            contJoints.position[1] = wrap_to_Pi(t) 
            #contJoints.position[1] = pendulum(t)

            joint_pub.publish(contJoints)
            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node