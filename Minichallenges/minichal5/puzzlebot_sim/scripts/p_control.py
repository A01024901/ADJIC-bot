#!/usr/bin/env python3 

import sys
import rospy  
from tqdm import tqdm
from geometry_msgs.msg import Twist   
from std_msgs.msg import Float32
from numpy import pi, sin, cos, arctan2, sqrt, exp

# This class will make the puzzlebot move following a square 
class LocClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################  
        r = 0.05        # wheel radius [m] 
        L = 0.19        # wheel separation [m] 
        self.d = 0      # distance
        self.dx = 0     # distance in x
        self.dy = 0     # distance in y
        self.theta = 0  # angle 
        vel = Twist()   # Robot's speed 
        self.wR = 0.0 
        self.wL = 0.0
        
        # Last activity
        self.f_v = 0.0
        self.f_w = 0.0
        goal_dx, goal_dy = 0.001, 6.0
        self.flag = "Go to Goal"
        
        ###******* INIT PUBLISHERS *******###  
        # pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  
        # self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        self.pub_gtg_vel = rospy.Publisher('gtg_vel', Twist, queue_size=1)
        self.pub_ed      = rospy.Publisher('ed', Float32, queue_size=1)
        self.pub_ed_theta      = rospy.Publisher('ed_theta', Float32, queue_size=1) 
        self.pub_goal_x     = rospy.Publisher('goal_x', Float32, queue_size=1)
        self.pub_goal_y     = rospy.Publisher('goal_y', Float32, queue_size=1)
        self.pub_xp         = rospy.Publisher('xp', Float32, queue_size=1)
        self.pub_yp         = rospy.Publisher('yp', Float32, queue_size=1)

        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("wl", Float32, self.wL_cb)  
        rospy.Subscriber("wr", Float32, self.wR_cb)  

        #********** INIT NODE **********# 
        freq = 50 
        rate = rospy.Rate(freq) # freq Hz  
        dT = 1/float(freq) #Dt is the time between one calculation and the next one 
        print("Node initialized " + str(freq) + " hz") 
        while not rospy.is_shutdown():
            v = r*(self.wR+self.wL)/2 
            w = r*(self.wR-self.wL)/L 
            self.d      = v*dT + self.d 
            self.dx     = v*dT*cos(self.theta) + self.dx
            self.dy     = v*dT*sin(self.theta) + self.dy
            self.theta  = w*dT + self.theta
            self.theta  = arctan2(sin(self.theta), cos(self.theta))
            self.etheta = arctan2(goal_dy-self.dy, goal_dx-self.dx) - self.theta
            self.etheta = arctan2(sin(self.etheta), cos(self.etheta))
            self.ed     = sqrt((goal_dx - self.dx)**2 + (goal_dy - self.dy)**2)
            edo = abs(self.ed)
            # Angular
            Kt = 0.5
            # Linear
            alpha, kmax = 1.0, 0.22
            Kl = kmax * ((1.0-exp(-alpha*(abs(edo))**2))/(abs(edo)))
            self.f_v = Kl * self.ed
            self.f_w = Kt * self.etheta
            vel = Twist()
            if (self.ed >= 0 and self.ed <= 0.20): self.flag = "STOP GR"  # 0.40
            else: vel.linear.x, vel.angular.z    = self.f_v, self.f_w
            if (self.flag == "STOP GR"): vel, self.f_v, self.f_w = Twist(), 0.0, 0.0    
            # pub goals and actual positions
            self.pub_goal_x.publish(goal_dx)
            self.pub_goal_y.publish(goal_dy)
            self.pub_xp.publish(self.dx)
            self.pub_yp.publish(self.dy)
            ###
            self.pub_gtg_vel.publish(vel) # publish the robot's speed
            self.pub_ed.publish(edo)      # publish the distance between robot and goal
            self.pub_ed_theta.publish(self.etheta)
            # self.pub_cmd_vel.publish(vel) #publish the robot's speed  
            print(f"""============================
GOAL X, Y:  {str(goal_dx)} , {str(goal_dy)}
flag:       {self.flag} 
F_v:        {str(self.f_v)}
F_w:        {str(self.f_w)}
etheta:     {str(self.etheta)}
ed:         {str(self.ed)}
============================""")

            rate.sleep()  
            if (self.flag == "STOP GR"): exit()

    def wL_cb(self, wL):  
        # This function receives a number   
        self.wL = wL.data 

    def wR_cb(self, wR):  
        # This function receives a number.  
        self.wR = wR.data  

    def cleanup(self):  
        # This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel = Twist()
        self.pub_gtg_vel.publish(vel) #publish the robot's speed  

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("loc", anonymous=True)  
    LocClass()
