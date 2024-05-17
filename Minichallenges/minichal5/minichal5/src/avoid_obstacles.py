#!/usr/bin/env python3 

import rospy 
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

np.set_printoptions(suppress = True)
np.set_printoptions(formatter = {'float':'{: 0.4f}'.format})

class PublishOdomCov:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('publish_odom_with_cov')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        #rospy.Subscriber("/wr" , Float32 , self.wr_cb)
        #rospy.Subscriber("/wl" , Float32 , self.wl_cb)

        ###--- Publishers ---###
        self.odom_pub = rospy.Publisher("odom" , Odometry , queue_size=1)

        self.dt = 0.02
        
        self.odom = Odometry()
        rate = rospy.Rate(int(1.0/self.dt))

        while rospy.get_time() == 0: print ("Simulacion no iniciada")#Descomentar en simulacion 

        print("Nodo operando")

        while not rospy.is_shutdown():
            mu_x = 1.0
            mu_y = 1.0
            mu_theta = 1.0

            sxx = 1.0
            syy = 1.0
            stheta = 1.0

            mu_v = 0.0
            mu_w = 0.0

            sigma2d_pose = np.array([[sxx , 0.0 , 0.0] , [0.0 , syy , 0.0] , [0.0 , 0.0 ,stheta]])
            print("2D Pose: " , (mu_x , mu_y , mu_theta))
            odom = self.fill_odom(mu_x , mu_y , mu_theta , sigma2d_pose , mu_v , mu_w)

            ###--- Publish ---###
            self.odom_pub.publish(self.odom)
            rate.sleep()

    def fill_odom(self , mu_x , mu_y , mu_theta , sigma2d_pose , mu_v , mu_w):
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.odom.header.stamp = rospy.Time.now()

        self.odom.pose.pose.position.x = mu_x
        self.odom.pose.pose.position.y = mu_y
        self.odom.pose.pose.position.z = 0
        quat = quaternion_from_euler(0 , 0 , mu_theta)
        self.odom.pose.pose.orientation.x = quat[0]
        self.odom.pose.pose.orientation.y = quat[1]
        self.odom.pose.pose.orientation.z = quat[2]
        self.odom.pose.pose.orientation.w = quat[3]

        self.odom.pose.covariance = [0.0] * 36

        self.odom.pose.covariance[0] = sigma2d_pose[0][0] #Covariance in x
        self.odom.pose.covariance[7] = sigma2d_pose[1][1] #Covariance in y 
        self.odom.pose.covariance[35] = sigma2d_pose[2][2] #Covariance in y 

        print("Fill covariance matrix")
        self.odom.twist.twist.linear.x = mu_v
        self.odom.twist.twist.angular.z = mu_w
        
    def wr_cb (self , msg):
        self.wr = msg.data

    def wl_cb (self , msg):
        self.wl = msg.data

    def cleanup (self):
        print ("Apagando Localsation")
        self.odom_pub.publish(Odometry())
        
        

if __name__ == "__main__": 
    PublishOdomCov()