#!/usr/bin/env python

# Simple class to turn the Fetch robot left


import rospy
from geometry_msgs.msg import Twist

class TurnLeft():
    def __init__(self):
        # initiliaze
        rospy.init_node('turn_left', anonymous=False)

        # Tell user how to stop Fetch
        rospy.loginfo("To stop Fetch CTRL + C")

        # ctrl + c to stop script
        rospy.on_shutdown(self.shutdown)
        
        # Create a publisher which can "talk" to Fetch and tell it to move
        pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
     
        #Fetch will stop if we don't keep telling it to move.
        r = rospy.Rate(10);

        # Twist is a datatype for linear and angular velocity
        move_cmd = Twist()
        # let's go forward at 0 m/s
        move_cmd.linear.x = 0
        # let's turn at 0.5 radians/s
        move_cmd.angular.z = 0.5

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            # publish the velocity
            pub.publish(move_cmd)
            # self.cmd_vel.publish(move_cmd)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
                        
        
    def shutdown(self):
        # stop Fetch
        rospy.loginfo("Stop Fetch")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop Fetch
        pub.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':

    try:
       TurnLeft()
    except:
        rospy.loginfo("GoForward node terminated.")
