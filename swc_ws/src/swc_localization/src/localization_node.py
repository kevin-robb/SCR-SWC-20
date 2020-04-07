#!/usr/bin/env python

import rospy
import math
from swc_msgs.msg import Control, Gps
from swc_msgs.srv import Waypoints

loc_pub = None
# robot's current GPS location
robot_gps = Gps()
# waypoints
start_gps = Gps()
bonus_gps = []
goal_gps = Gps()

def interpret_waypoints(waypoints):
    global start_gps, bonus_gps, goal_gps
    # first waypoint is start location
    start_gps = waypoints[0]
    # next three waypoints are bonuses
    bonus_gps.append(waypoints[1])
    bonus_gps.append(waypoints[2])
    bonus_gps.append(waypoints[3])
    # last waypoint is goal location
    goal_gps = waypoints[4]

def get_gps_reading(gps_reading):
    global robot_gps
    robot_gps = gps_reading

def timer_callback(event): #TODO might not even need timer and can just run all this in get_gps_reading
    # Create a GPS object that will store the relative locaiton of the goal
    # TODO Can later change this to target the nearest unvisited bonus waypoint
    rel_gps = Gps()
    rel_gps.latitude = goal_gps.latitude - robot_gps.latitude
    rel_gps.longitude = goal_gps.longitude - robot_gps.longitude

    # Publish the relative location of the goal
    loc_pub.publish(rel_gps)


def main():
    global loc_pub

    # Initalize our node in ROS
    rospy.init_node('localization_node')

    # Create a Publisher that we can use to publish messages to the /swc/goal topic
    loc_pub = rospy.Publisher("/swc/goal", Gps, queue_size=1)

    # Create a Subscriber that will get the robot's current GPS position
    gps_sub = rospy.Subscriber("/sim/gps", Gps, get_gps_reading, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service('/sim/waypoints')
    waypoints = rospy.ServiceProxy('/sim/waypoints', Waypoints)()
    # Create GPS variables of the waypoints
    interpret_waypoints(waypoints)
    #print(waypoints.waypoints)

    # Create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
