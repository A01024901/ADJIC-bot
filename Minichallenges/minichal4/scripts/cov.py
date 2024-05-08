#!/usr/bin/env python3 

import rospy

import numpy as np

from geometry_msgs.msg import Twist  

#This class will publish the speed to the /cmd_vel topic to make the robot move for some period of time 

# Then stops  

class worked_example():  

    def __init__(self):  

        # first thing, init a node! 

        rospy.init_node('worked_example')  

        # Initialize the variables

        meas_0 = np.array([[0], [0], [0]]) # Initial position of the robot [x, y, theta]
        meas = np.array([[0], [0], [0]]) # Previous position of the robot [x, y, theta]
        cov_0 =np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]) # Initial covariance matrix
        cov = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]) # Covariance matrix
        h = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]) # Jacobian of the measurement model
        qk = np.array([[0.5, 0.01, 0.01], [0.01, 0.5, 0.01], [0.01, 0.01, 0.2]]) # Process noise covariance matrix
        vk = 1 #m/s
        wk = 1 #rad/s
        dt = 0.1 #s


        # Main loop

        for i in range(2): 
                
                meas = np.array([[meas_0[0,0] + dt * vk * np.cos(meas_0[2,0])], [meas_0[1,0] + dt * vk * np.sin(meas_0[2,0])], [meas_0[2,0] + dt * wk]]) # Prediction step
                h = np.array([[1, 0, -dt * vk * np.sin(meas_0[2,0])], [0, 1, dt * vk * np.cos(meas_0[2,0])], [0, 0, 1]]) # Jacobian of the measurement model
                cov = h.dot(cov_0).dot(h.T) + qk # Covariance matrix update

                meas_0 = meas
                cov_0 = cov
            
            
                print("meas_0:")
                print("[{:.4f}]".format(meas_0[0,0]))
                print("[{:.4f}]".format(meas_0[1,0]))
                print("[{:.4f}]".format(meas_0[2,0]))

                print("h_0:")
                print("[{:.4f} {:.4f} {:.4f}]".format(h[0,0], h[0,1], h[0,2]))
                print("[{:.4f} {:.4f} {:.4f}]".format(h[1,0], h[1,1], h[1,2]))
                print("[{:.4f} {:.4f} {:.4f}]".format(h[2,0], h[2,1], h[2,2]))

                print("cov_0:")
                print("[{:.4f} {:.4f} {:.4f}]".format(cov_0[0,0], cov_0[0,1], cov_0[0,2]))
                print("[{:.4f} {:.4f} {:.4f}]".format(cov_0[1,0], cov_0[1,1], cov_0[1,2]))
                print("[{:.4f} {:.4f} {:.4f}]".format(cov_0[2,0], cov_0[2,1], cov_0[2,2]))

            
############################### MAIN PROGRAM ####################################  

if __name__ == "__main__": 

    worked_example()


    