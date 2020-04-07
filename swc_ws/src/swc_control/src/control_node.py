#!/usr/bin/env python

import rospy
import math
from swc_msgs.msg import Control, Gps

control_pub = None

def get_desired_location(rel_goal_loc_gps):
    # TODO figure out how to get to the desired location and turn that into commands
    # (possibly in a different, new node)
    goal_gps = rel_goal_loc_gps

def timer_callback(event):
    # TODO make an actual command using the desired relative location from /swc/goal
    control_msg = Control()
    #control_msg.speed = 1
    #control_msg.turn_angle = 15

    # Publish the message to /sim/control so the simulator receives it
    control_pub.publish(control_msg)

def main():
    global control_pub

    # Initalize our node in ROS
    rospy.init_node('control_node')

    # Create a Publisher that we can use to publish messages to the /sim/control topic
    control_pub = rospy.Publisher("/sim/control", Control, queue_size=1)

    # Create a Subscriber that will get the relative desired location
    goal_sub = rospy.Subscriber("/swc/goal", Gps, get_desired_location, queue_size=1)

    # Create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
