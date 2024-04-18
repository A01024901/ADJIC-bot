#!/usr/bin/env python3 

import rospy 
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

class tf_broadcaster:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('tf2_broadcaster')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        #rospy.Subscriber("/cmd_vel" , Twist , self.vel_cb)

        ###--- Broadcaster ---###
        self.tf_broadcast = tf2_ros.TransformBroadcaster()

        ###--- Transformation ---###
        self.t = TransformStamped()

        ###--- Robot Pose ---###
        self.x = 1.0
        self.y = 0.0
        self.theta = 0.0
        
        rate = rospy.Rate(50)

        #while rospy.get_time() == 0: print ("Simulacion no iniciada")#Descomentar en simulacion 

        print("Broadcast operando")
        print("Use rviz to see rotation")

        while not rospy.is_shutdown():
            
            # Fill the transformation information 

            self.t.header.stamp = rospy.Time.now() 

            self.t.header.frame_id = "odom" 

            self.t.child_frame_id = "base_link" 

            self.t.transform.translation.x = self.x 

            self.t.transform.translation.y = self.y 

            self.t.transform.translation.z = 0.0 

            self.theta += 0.01 #theta will be increasing 

            #Clip the value of theta to the interval [-pi to pi] 

            theta = np.arctan2(np.sin(self.theta), np.cos(self.theta)) 

            #The transformation requires the orientation as a quaternion 

            q = quaternion_from_euler(0, 0, self.theta) 

            self.t.transform.rotation.x = q[0] 

            self.t.transform.rotation.y = q[1] 

            self.t.transform.rotation.z = q[2] 

            self.t.transform.rotation.w = q[3] 

            # A transformation is broadcasted instead of published 

            self.tf_broadcast.sendTransform(self.t) #broadcast the transformation 

            rate.sleep()

    def rotate_wheel(self):
        pass


    def cleanup (self):
        print ("Esta transmision se ha terminado")
        

if __name__ == "__main__": 
    tf_broadcaster()