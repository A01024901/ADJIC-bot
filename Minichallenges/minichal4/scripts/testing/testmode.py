#!/usr/bin/env python3 

import rospy 
import numpy as np
import time
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler

class test_mode:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('adjic_test_mode')
        rospy.on_shutdown(self.cleanup)

        print("\n \n")

        ###--- Subscriptores ---###
        #rospy.Subscriber("/cmd_vel" , Twist , self.vel_cb)

        ###--- Publishers ---###
        self.pose_pub = rospy.Publisher("pose_sim" , PoseStamped , queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel" , Twist , queue_size=1)
        self.wr_pub = rospy.Publisher("wr" , Float32 , queue_size=1)
        self.wl_pub = rospy.Publisher("wl" , Float32 , queue_size=1)

        ###--- Constants ---###
        self.dt = 0.2

        ###--- Variables ---####
        self.flag = False

        self.l_vel = 0.0
        self.w_vel = 0.0 
        self.t = 0.0
        self.angle = 0.0
        self.pos = 0.0 

        rate = rospy.Rate(int(1.0/self.dt))

        self.cmd_vel = Twist()

        #while rospy.get_time() == 0: print ("Simulacion no iniciada")#Descomentar en simulacion 

        print("Simulacion operando")

        while not rospy.is_shutdown():
            
            self.test_pose()
            
            rate.sleep()

    def test_pose (self):
        self.l_vel = float(input("Dame la velocidad lineal (m/s): "))
        deg_per_sec = float(input("Dame la velocidad angular(g/s): "))
        self.w_vel = math.radians(deg_per_sec)
        self.t = float(input("Dame la duracion en segundos: "))
        x = (self.l_vel * self.t) * np.cos(self.w_vel * self.t)
        y = (self.l_vel * self.t) * np.sin(self.w_vel * self.t)
        theta = self.w_vel * self.t
        print(f"Linear: {self.l_vel}  Angular: {self.w_vel} Time: {self.t}", (self.l_vel , self.w_vel , self.t))
        print(f" Pos X: {x} Pos Y: {y} Theta: {theta}. Estas seguro?(t/f)", (x , y , theta))
        flag = input("Estas seguro?(t/f)")
        if ("t" in flag.lower()):
            self.cmd_vel.linear.x = self.l_vel
            self.cmd_vel.angular.z = self.w_vel
            self.cmd_vel_pub.publish(self.cmd_vel)
            time.sleep(self.t)
            self.cmd_vel.linear.x = 0
            self.cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(self.cmd_vel)
            print("\n \n")

        else: pass

        


    def cleanup (self):
        print ("Apagando Simulacion")
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel)
        

if __name__ == "__main__": 
    test_mode()