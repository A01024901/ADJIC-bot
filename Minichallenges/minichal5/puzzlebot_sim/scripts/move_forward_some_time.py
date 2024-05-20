#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist

# This class will subscribe to the /number topic and display the received number as a string message
class MoveFClass():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        ###******* INIT PUBLISHERS *******###
        # create the publisher to cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # create a twist message, fill in the details
        my_twist = Twist()
        
        rate = rospy.Rate(10)  # 10 hz
        
        rospy.loginfo("About to be moving forward!")
        n = 0
        while not rospy.is_shutdown():
            if n < 50:  # If rate is 10Hz we move 50.0 * 0.1s = 5s
                # rospy.loginfo("moving forward!")
                # Fill in the message with required data
                my_twist.linear.x = 0.2  # Our forward speed in [m/s]. (0.2[m/s] * 5[s]) = 1[m]
                my_twist.angular.z = 0   # Our angular speed in [rad/s], (In this case the robot does not rotate) math.pi/10
            
            else:
                # rospy.loginfo("stop!")
                my_twist.linear.x = 0.0  # Our forward speed in [m/s]
                my_twist.angular.z = 0.0
                
            # Send the speed to the robot
            self.cmd_vel_pub.publish(my_twist)
            n = n + 1
            
            # Wait enough time to keep the required rate (10Hz)
            rate.sleep()            
            

    def cleanup(self):
        # This function is called just before finishing the node
        # You can use it to clean things up before leaving
        # Example: stop the robot before finishing a node.
        message = Twist()
        self.cmd_vel_pub.publish(message)
        print("Stopping the robot")
        print("Bye bye!!!")
        

############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    # First thing, init a node!
    rospy.init_node("move_forward_1m")
    MoveFClass()

