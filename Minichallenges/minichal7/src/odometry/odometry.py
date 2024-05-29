#!/usr/bin/env python3 
import rospy 
from dead_reckoning_class import dead_reckoning
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class localisation:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('localisation')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/wr" , Float32 , self.wr_cb)
        rospy.Subscriber("/wl" , Float32 , self.wl_cb)
        rospy.Subscriber("/ar_x" , Float32 , self.arx_cb)
        rospy.Subscriber("/ar_y" , Float32 , self.ary_cb)
        rospy.Subscriber("/arucos_flag" , Bool , self.flag_cb)

        ###--- Publishers ---###
        self.odom_pub = rospy.Publisher("odom" , Odometry , queue_size=1)

        ###--- Robot Constants ---###
        self.r = 0.05
        self.l = 0.19
        self.dt = 0.02

        ###--- Variables ---####
        self.v = 0.0
        self.w = 0.0
        self.wr = 0.0 
        self.wl = 0.0
        self.xa = 0.0
        self.ya = 0.0
        self.flag = False
        
        
        self.odom = Odometry()
        self.covariance = dead_reckoning(self.dt)
        rate = rospy.Rate(int(1.0/self.dt))

        #while rospy.get_time() == 0: print ("Simulacion no iniciada")#Descomentar en simulacion 

        print("Nodo operando")

        while not rospy.is_shutdown():
            self.get_robot_velocities()
            self.update_robot_pose()
            cov_mat , u = self.covariance.calculate(self.v , self.w , self.wr , self.wl)
            self.get_odom(cov_mat , u)

            ###--- Publish ---###
            self.odom_pub.publish(self.odom)
            rate.sleep() 

    def get_robot_velocities (self):
        self.v = (self.r * (self.wr + self.wl))/2
        self.w = self.r * ((((2*self.v/self.r) - self.wl)-self.wl)/self.l)

    def get_odom (self , cov_mat , u): 
        self.odom.header.frame_id = "odom"
        self.odom.pose.pose.position.x = u[0] #+ 0.1
        self.odom.pose.pose.position.y = u[1]

        quat = quaternion_from_euler(0 , 0 , u[2])
        self.odom.pose.pose.orientation.x = quat[0]
        self.odom.pose.pose.orientation.y = quat[1]
        self.odom.pose.pose.orientation.z = quat[2]
        self.odom.pose.pose.orientation.w = quat[3]

        self.odom.pose.covariance = [0.0] * 36

        self.odom.pose.covariance[0] = cov_mat[0][0] * 17 #Covariance in x
        self.odom.pose.covariance[1] = cov_mat[0][1] #Covariance in xy 
        self.odom.pose.covariance[5] = cov_mat[0][2] #Covariance in x theta
        self.odom.pose.covariance[6] = cov_mat[1][0] #Covariance in y x
        self.odom.pose.covariance[7] = cov_mat[1][1] #Covariance in y 
        self.odom.pose.covariance[11] = cov_mat[1][2] #Covariance in y theta
        self.odom.pose.covariance[30] = cov_mat[2][0] #Covariance in theta x
        self.odom.pose.covariance[31] = cov_mat[2][1] #Covariance in theta y
        self.odom.pose.covariance[35] = cov_mat[2][2] * 5 #Covariance in theta
        
    def wr_cb (self , msg):
        self.wr = msg.data

    def wl_cb (self , msg):
        self.wl = msg.data

    def arx_cb (self , msg):
        self.xa = msg.data

    def ary_cb (self , msg):
        self.ya = msg.data

    def flag_cb (self , msg):
        self.flag = msg.data

    def cleanup (self):
        print ("Apagando Localsation")
        self.odom_pub.publish(Odometry())
        
        

if __name__ == "__main__": 
    localisation()