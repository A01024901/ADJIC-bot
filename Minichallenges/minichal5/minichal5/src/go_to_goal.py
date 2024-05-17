#!/usr/bin/env python 
import rospy  
import numpy as np 
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32 
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


#This class will make the puzzlebot move following a square 

class GoToGoal:  
    def __init__(self):  
        ###--- Inicio del Nodo ---###
        rospy.init_node('go_to_goal')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/puzzlebot_1/wl", Float32, self.wl_cb)  
        rospy.Subscriber("/puzzlebot_1/wr", Float32, self.wr_cb)
        #rospy.Subscriber("target", PoseStamped, self.wr_cb)  

        ###--- Publishers ---###
        self.pub_cmd_vel = rospy.Publisher('gtg_twist', Twist, queue_size=1)  
        self.pos_pub = rospy.Publisher('pos_gtg', PoseStamped, queue_size=1)
        self.pos_t_pub = rospy.Publisher('target_gtg', PoseStamped, queue_size=1)

        ###--- Constants ---###
        self.dt = 0.02
        self.r = 0.05 #wheel radius [m] 
        self.L = 0.19 #wheel separation [m] 

        ###--- Objetos ---###
        self.pose = PoseStamped()
        self.pose_target = PoseStamped()
        self.v_msg = Twist() 
        rate = rospy.Rate(int(1.0/self.dt))

        ###--- Variables ---###
        self.x_pos = 0.0 
        self.y_pos = 0.0
        self.x_target = 2.0 
        self.y_target = 2.0 
        self.angle = 0.0
        self.wl = 0.0 
        self.wr = 0.0

        while rospy.get_time() == 0: print ("Simulacion no iniciada") #Descomentar en simulacion 

        self.t_ant = rospy.get_time() #Valor para obtener dt 

        while not rospy.is_shutdown(): 
            self.calc_pos()
            v , w = self.calc_gtg()
            self.fill_messages(v , w)

            self.pub_cmd_vel.publish(self.v_msg) #publish the robot's speed  
            self.pos_pub.publish(self.pose) #publish robot position
            self.pos_t_pub.publish(self.pose_target) #Publish target position

            rate.sleep() 

    def calc_pos(self):
        dt = rospy.get_time() - self.t_ant
        self.t_ant = rospy.get_time()

        v = self.r * (self.wr - self.wl)/2 #Velocidad linear
        w = self.r * (self.wr - self.wl)/self.L   #Velocidad angular 

        self.angle = self.angle + w * dt #Theta 
        self.angle = np.arctan2(np.sin(self.angle) , np.cos(self.angle))
        self.x_pos = self.x_pos + v * dt * np.cos(self.angle) #Posicion en X
        self.y_pos = self.y_pos + v * dt * np.sin(self.angle) #Posicion en Y

    def calc_gtg (self):
        kvmax = 0.16 #0.17 #linear speed maximum gain 
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
    
    def fill_messages(self , v , w ) :
        self.v_msg.linear.x = v
        self.v_msg.angular.z = w
        self.pose.pose.position.x = self.x_pos
        self.pose.pose.position.y = self.y_pos
        self.pose_target.pose.position.x = self.x_target
        self.pose_target.pose.position.y = self.y_target

    def wl_cb(self, wl):  
        self.wl = wl.data 

    def wr_cb(self, wr): 
        self.wr = wr.data

    #def target_cb(self , target):
    #    self.x_target = target.data[0]
    #    self.y_target = target.data[1]   

    def cleanup(self):  
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

if __name__ == "__main__":  
    GoToGoal()  