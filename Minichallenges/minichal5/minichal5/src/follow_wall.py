#!/usr/bin/env python3 

import rospy 
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan  
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler

np.set_printoptions(suppress = True)
np.set_printoptions(formatter = {'float':'{: 0.4f}'.format})

class PublishOdomCov:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('follow_wall')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/puzzlebot_1/scan", LaserScan, self.laser_cb)  

        ###--- Publishers ---###
        self.pub_cmd_vel = rospy.Publisher('gtg_twist', Twist, queue_size=1) 

        ###--- Constants ---###
        self.dt = 0.02

        ###--- Objetos ---###
        self.cmd_vel = Twist() 
        rate = rospy.Rate(int(1.0/self.dt))

        ###--- Variables ---###
        self.scan = []

        while rospy.get_time() == 0: print ("Simulacion no iniciada") 

        while not rospy.is_shutdown():

            ###--- Publish ---###
            self.odom_pub.publish(self.odom)
            rate.sleep()

    def fill_odom(self , mu_x , mu_y , mu_theta , sigma2d_pose , mu_v , mu_w):
        pass 

    def laser_cb (self , msg):
        self.scan = msg.data

    def wl_cb (self , msg):
        self.wl = msg.data

    def cleanup (self):
        print ("Apagando Localsation")
        self.odom_pub.publish(Odometry())
        
        

if __name__ == "__main__": 
    PublishOdomCov()