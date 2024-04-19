#!/usr/bin/env python3 

import rospy 
import tf2_ros
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState 
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

class wheels_rotation:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('wheels_rotation')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscribers ---###
        rospy.Subscriber("wr" , Float32 , self.wr_cb)
        rospy.Subscriber("wl" , Float32 , self.wl_cb)

        ###--- Publisher ---###
        joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1) 

        ###--- Joints ---###
        self.contJoints = JointState() 
        
        ###--- Variables Ruedas ---###
        self.wr = 0
        self.wl = 0
        self.contJoints.header.frame_id = "base_link" 
        self.contJoints.name.extend(["wl_joint", "wr_joint"]) 
        self.contJoints.position.extend([0.0, 0.0]) 
        self.contJoints.velocity.extend([0.0, 0.0]) 
        self.contJoints.effort.extend([0.0, 0.0]) 

        ###--- Robot Pose ---###
        rate = rospy.Rate(30)

        print("Broadcast operando")
        print("Use rviz to see rotation")

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() 
            self.contJoints.header.stamp = rospy.Time.now() 
            self.contJoints.position[0] = self.wrap_to_Pi(self.wl * t) 
            self.contJoints.position[0] += self.wr / t

            self.contJoints.position[1] = self.wrap_to_Pi(self.wr * t)  
            joint_pub.publish(self.contJoints) 
            rate.sleep()

    def wrap_to_Pi(self , theta): 
        result = np.fmod((theta + np.pi),(2 * np.pi)) 
        if(result < 0): 
            result += 2 * np.pi 
        return result - np.pi 
    
    def wr_cb(self , msg):
        self.wr = msg.data

    def wl_cb(self , msg):
        self.wl = msg.data

    def cleanup (self):
        print ("Esta transmision se ha terminado")    

if __name__ == "__main__": 
    wheels_rotation()