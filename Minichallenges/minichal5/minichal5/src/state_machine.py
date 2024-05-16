#!/usr/bin/env python3 

import rospy 
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

np.set_printoptions(suppress = True)
np.set_printoptions(formatter = {'float':'{: 0.4f}'.format})

class state_machine:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('state_machine')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("gtg_twist" , Twist , self.gtg_cb)
        rospy.Subscriber("pos_gtg" , Twist , self.gtg_cb)
        rospy.Subscriber("fw_twist" , Twist , self.fw_cb)

        ###--- Publishers ---###
        self.cmd_vel_pub = rospy.Publisher("puzzlebot_1/base_controller/cmd_vel" , Twist , queue_size=1)

        ###--- Variables ---###
        self.pos = 0
        self.state = "go_to_goal" #"avoid_obstacle" "Stop"

        ###--- Constants ---###
        self.dt = 0.02

        ###--- Objects ---###
        self.cmd_vel = Twist()
        self.gtg_vel = Twist()
        self.fw_vel = Twist()
        rate = rospy.Rate(int(1.0/self.dt))

        while rospy.get_time() == 0: print ("Simulacion no iniciada") #Descomentar en simulacion 

        print("Nodo operando")

        while not rospy.is_shutdown():
            if self.state == "on_hold":
                print ("On_hold")   

            if self.state == "go_to_goal":
                print ("Go to goal")
                self.gtg_behave()

            elif self.state == "follow_wall":
                print ("Follow Wall")
                self.follow_wall_behave()

            elif self.state == "stop":
                print ("Reached goal")
                self.stop_behave()

            ###--- Publish ---###
            self.cmd_vel_pub.publish(self.cmd_vel)
            rate.sleep()

    def hold_behave(self):
        if self.pos[0] != self.pos[2] and self.pos[1] != self.pos[3]:
            self.state = "go_to_goal"

    def gtg_behave(self):
        self.cmd_vel = self.gtg_vel
        if self.pos[0] != self.pos[2] and self.pos[1] != self.pos[3]:
            self.state = "stop"

    def follow_wall_behave(self):
        self.cmd_vel = self.fw_vel

    def stop_behave(self):
        pass

    def gtg_cb(self , twist):
        self.gtg_vel = twist

    def pos_gtg_cb(self , msg):
        self.pos = msg

    def fw_cb(self , twist):
        self.fw_vel = twist   

    def cleanup (self):
        print ("Apagando Localsation")
        
        

if __name__ == "__main__": 
    state_machine()