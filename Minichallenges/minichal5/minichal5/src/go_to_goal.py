#!/usr/bin/env python 
import rospy  
import numpy as np 
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32 
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class GoToGoal:  
    def __init__(self):  
        ###--- Inicio del Nodo ---###
        rospy.init_node('go_to_goal')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("odom", Odometry, self.odom_cb)  
        #rospy.Subscriber("target", PoseStamped, self.wr_cb)  

        ###--- Publishers ---###
        self.pub_cmd_vel = rospy.Publisher('gtg_twist', Twist, queue_size=1)  
        self.pos_pub = rospy.Publisher('pos_gtg', PoseStamped, queue_size=1)
        self.pos_t_pub = rospy.Publisher('target_gtg', PoseStamped, queue_size=1)
        self.at_goal_flag_pub = rospy.Publisher('at_goal_flag', Bool, queue_size=1)

        ###--- Constants ---###
        self.dt = 0.02
        self.r = 0.05 #wheel radius [m] 
        self.L = 0.19 #wheel separation [m] 

        ###--- Objetos ---###
        self.flag_msg = Bool()
        self.pose = PoseStamped()
        self.pose_target = PoseStamped()
        self.v_msg = Twist() 
        rate = rospy.Rate(int(1.0/self.dt))

        ###--- Variables ---###
        self.at_goal_flag = False
        self.x_pos = 0.0 
        self.y_pos = 0.0
        self.x_target = 5
        self.y_target = 0.
        self.angle = 0.0

        while rospy.get_time() == 0: print ("Simulacion no iniciada") #Descomentar en simulacion 

        self.t_ant = rospy.get_time() #Valor para obtener dt 

        while not rospy.is_shutdown(): 
            v , w = self.calc_gtg()
            self.at_goal()
            self.fill_messages(v , w)

            self.pub_cmd_vel.publish(self.v_msg) #publish the robot's speed  
            self.pos_pub.publish(self.pose) #publish robot position
            self.pos_t_pub.publish(self.pose_target) #Publish target position
            self.at_goal_flag_pub.publish(self.flag_msg) #Publish flag
            
            rate.sleep() 

    def calc_gtg (self):
        kvmax = 0.17 #0.17 #linear speed maximum gain 
        kwmax = 0.8#0.8 #angular angular speed maximum gain
        
        av = 2.0 #Constant to adjust the exponential's growth rate  
        aw = 2.0 #Constant to adjust the exponential's growth rate
        ed = np.sqrt((self.x_target - self.x_pos)**2+(self.y_target - self.y_pos)**2)
        #Compute angle to the target position
        theta_target=np.arctan2(self.y_target - self.y_pos , self.x_target - self.x_pos)
        e_theta = theta_target - self.angle

        #limit e_theta from -pi to pi
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        #Compute the robot's angular speed
        if e_theta != 0:
            kw = kwmax * (1 - np.exp(-aw * e_theta **2))/abs(e_theta) #Constant to change the speed 
        else:
            kw = 0.05#0.08 #0.05
        w = kw * e_theta
        if abs(e_theta) > np.pi/8:
            #we first turn to the goal
            v = 0 #linear speed 
        else:
            # Make the linear speed gain proportional to the distance to the target position
            kv = kvmax * (1 - np.exp(-av * ed **2))/abs(ed) #Constant to change the speed 
            v = kv * ed #linear speed

        return v , w
    
    def at_goal(self):
        x_window = self.x_pos < (self.x_target + 0.1) and self.x_pos > (self.x_target - 0.1)
        y_window = self.y_pos < (self.y_target + 0.1) and self.y_pos > (self.y_target - 0.1)
        if x_window and y_window:
            self.at_goal_flag = True

        else: self.at_goal_flag = False
    
    def fill_messages(self , v , w ) :
        self.v_msg.linear.x = v
        self.v_msg.angular.z = w
        self.pose.pose.position.x = self.x_pos
        self.pose.pose.position.y = self.y_pos
        self.pose_target.pose.position.x = self.x_target
        self.pose_target.pose.position.y = self.y_target

        self.flag_msg.data = self.at_goal_flag

    def odom_cb(self , msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
    
    #def target_cb(self , target):
    #    self.x_target = target.data[0]
    #    self.y_target = target.data[1]   

    def cleanup(self):  
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

if __name__ == "__main__":  
    GoToGoal()  