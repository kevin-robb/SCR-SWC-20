#!/usr/bin/env python

import rospy
from math import degrees
from std_msgs.msg import Float32
from swc_msgs.msg import Control

control_pub = None

# TODO check for obstacles to override commands with reactive avoidance

def get_turn_angle(turn):
    control_msg = Control()
    control_msg.speed = 1.5 # for now, keep speed constant at 1
    control_msg.turn_angle = degrees(turn.data) # turn.data is in radians
    control_pub.publish(control_msg)

def main():
    global control_pub

    # Initalize our node in ROS
    rospy.init_node('control_node')

    # Create publisher for the command messages
    control_pub = rospy.Publisher("/sim/control", Control, queue_size=1)

    # Create subscriber for nav's commanded turn angle
    turn_sub = rospy.Subscriber("/swc/turn_cmd", Float32, get_turn_angle, queue_size=1)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
