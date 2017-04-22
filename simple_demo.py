#!/usr/bin/env python

import copy
import actionlib # To implement an ActionServer
import rospy

from math import sin, cos

from geometry_msgs.msg import PoseStamped # Imports Point and Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Move base using the navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        # The target_pose is the goal that the navigation stack attempts to achieve
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y

        # This represents an orientation in free space in quaternion form.
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)

        #Frame this data is associated with
        # 0: no frame
        # 1: global frame
        move_goal.target_pose.header.frame_id = frame
        #Two-integer timestamp that is expressed as:
        # * stamp.secs: seconds (stamp_secs) since epoch
        # * stamp.nsecs: nanoseconds since stamp_secs
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work ????
        self.client.send_goal(move_goal)
        self.client.wait_for_result()
        
if __name__ == "__main__":
    # Create a node
    rospy.init_node("fetch_demo")

    # Check to see if sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()
    
 # Move the base to a location using the nav stack
    
    # Move to the first table
    rospy.loginfo("Moving to Table_0")
    move_base.goto(1.0, 1.2, 1.7) # move_base.goto(x,y,theta)
    rospy.loginfo("Sleeping 5 secs")
    rospy.sleep(5.)
    # Move to the second table
    rospy.loginfo("Moving to Table")
    move_base.goto(1.0, -1.2, 4.8)

