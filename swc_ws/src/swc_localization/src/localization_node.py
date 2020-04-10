#!/usr/bin/env python

import rospy
from math import pi
from tf import transformations
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
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
    start_gps = waypoints.waypoints[0]
    # next three waypoints are bonuses
    bonus_gps.append(waypoints.waypoints[1])
    bonus_gps.append(waypoints.waypoints[2])
    bonus_gps.append(waypoints.waypoints[3])
    # last waypoint is goal location
    goal_gps = waypoints.waypoints[4]

def update_gps(gps_reading):
    global robot_gps
    robot_gps = gps_reading

    # Create a GPS object that will store the relative location of the goal
    # TODO Can later change this to target the nearest unvisited bonus waypoint
    rel_gps = Gps()
    rel_gps.latitude = goal_gps.latitude - robot_gps.latitude
    rel_gps.longitude = goal_gps.longitude - robot_gps.longitude
    # Publish the relative location of the goal
    loc_pub.publish(rel_gps)
    # TODO DEBUG
    #print(rel_gps)

# similar function to beebotics' reactive_node.py/get_current_heading()
def update_heading(imu_data):
    # returns robot's current heading in radians (0 north, CW)
    orientation = imu_data.orientation
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w)
    yaw_rads = transformations.euler_from_quaternion(quaternion)[2]

    current_heading = Float32()
    current_heading.data = yaw_rads + pi
    hdg_pub.publish(current_heading)

def main():
    global loc_pub, hdg_pub

    # Initalize our node in ROS
    rospy.init_node('localization_node')

    # Create publishers for the relative goal location and the robot's current heading
    loc_pub = rospy.Publisher("/swc/goal", Gps, queue_size=1)
    hdg_pub = rospy.Publisher("/swc/current_heading", Float32, queue_size=1)

    # Create subscribers for robot's current GPS position and IMU data
    gps_sub = rospy.Subscriber("/sim/gps", Gps, update_gps, queue_size=1)
    imu_sub = rospy.Subscriber("/sim/imu", Imu, update_heading, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service('/sim/waypoints')
    waypoints = rospy.ServiceProxy('/sim/waypoints', Waypoints)()
    # Create GPS variables of the waypoints
    interpret_waypoints(waypoints)
    #print(waypoints.waypoints)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
