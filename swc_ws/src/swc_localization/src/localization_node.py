#!/usr/bin/env python

import rospy
from math import pi, sqrt
from tf import transformations
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from swc_msgs.msg import Control, Gps
from swc_msgs.srv import Waypoints

loc_pub = None
hdg_pub = None
pts_pub = None
# robot's current GPS location
robot_gps = Gps()
# waypoints
start_gps = Gps()
bonus_gps = []
goal_gps = Gps()
wp_interpreted = False
# keep track of which waypoints have been visited
visited = [False, False, False]
error_margin = 0.00001

def interpret_waypoints(waypoints):
    global start_gps, bonus_gps, goal_gps, visited, wp_interpreted
    # first waypoint is start location
    start_gps = waypoints.waypoints[0]
    # next three waypoints are bonuses
    bonus_gps.append(waypoints.waypoints[1])
    bonus_gps.append(waypoints.waypoints[2])
    bonus_gps.append(waypoints.waypoints[3])
    # last waypoint is goal location
    goal_gps = waypoints.waypoints[4]
    # initialize, no points have been visited yet
    visited = [False, False, False]
    wp_interpreted = True
    print("waypoints interpreted")

def arrived_at_point(point_gps):
    #dist = sqrt((point_gps.latitude - robot_gps.latitude)**2 + (point_gps.longitude - robot_gps.longitude)**2)
    if point_gps.latitude - robot_gps.latitude < error_margin and point_gps.longitude - robot_gps.longitude < error_margin:
        return True
    else:
        return False

def update_robot_gps(gps_reading):
    global robot_gps, visited
    robot_gps = gps_reading
    print("got robot gps")
    # check all the bonus waypoints to see if visited.
    # make sure the waypoints have been interpreted first.
    if wp_interpreted:
        #print("checking if points visited")
        if not visited[0]:
            if arrived_at_point(bonus_gps[0]):
                visited[0] = True
                print("arrived at point 1")
        if not visited[1]:
            if arrived_at_point(bonus_gps[1]):
                visited[1] = True
                print("arrived at point 2")
        if not visited[2]:
            if arrived_at_point(bonus_gps[2]):
                visited[2] = True
                print("arrived at point 3")
        # make the list of not-yet-visited points' relative positions
        make_rel_pt_list()

def make_rel_pt_list():
    # make a list of the three bonus points and the goal, all relative to the starting point.
    # only include non-visited points.
    # this will be used for path planning.
    point_list = []
    # bonus waypoint 1
    if not visited[0]:
        print("going for point 1")
        rel_b1 = Gps()
        rel_b1.latitude = bonus_gps[0].latitude - robot_gps.latitude
        rel_b1.longitude = bonus_gps[0].longitude - robot_gps.longitude
        point_list.append(rel_b1)
    # bonus waypoint 2
    elif not visited[1]:
        print("going for point 2")
        rel_b2 = Gps()
        rel_b2.latitude = bonus_gps[1].latitude - robot_gps.latitude
        rel_b2.longitude = bonus_gps[1].latitude - robot_gps.longitude
        point_list.append(rel_b2)
    # bonus waypoint 3
    elif not visited[2]:
        print("going for point 3")
        rel_b3 = Gps()
        rel_b3.latitude = bonus_gps[2].latitude - robot_gps.latitude
        rel_b3.longitude = bonus_gps[2].latitude - robot_gps.longitude
        point_list.append(rel_b3)
    # final goal waypoint 
    else:
        print("going for final goal")
        rel_goal = Gps()
        rel_goal.latitude = goal_gps.latitude - robot_gps.latitude
        rel_goal.longitude = goal_gps.latitude - robot_gps.longitude
        point_list.append(rel_goal)
    # send next point as the current target position
    loc_pub.publish(point_list[0])

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
    global loc_pub, hdg_pub, pts_pub

    # Initalize our node in ROS
    rospy.init_node('localization_node')

    # publish target point location relative to robot's current position
    loc_pub = rospy.Publisher("/swc/goal", Gps, queue_size=1)
    # publish robot's current heading
    hdg_pub = rospy.Publisher("/swc/current_heading", Float32, queue_size=1)
    # publish list of relative GPS positions of not-yet-visited points, including final goal
    #pts_pub = rospy.Publisher("/swc/rel_waypoints", Waypoints, queue_size=1)

    # subscribe to robot's current GPS position and IMU data
    gps_sub = rospy.Subscriber("/sim/gps", Gps, update_robot_gps, queue_size=1)
    imu_sub = rospy.Subscriber("/sim/imu", Imu, update_heading, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service('/sim/waypoints')
    waypoints = rospy.ServiceProxy('/sim/waypoints', Waypoints)()
    # Create GPS variables of the waypoints
    interpret_waypoints(waypoints)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



# def make_rel_pt_list():
#     # send a list of the three bonus points and the goal, all relative to the starting point
#     # this will be used for path planning
#     point_list = Waypoints()
#     point_list.waypoints = []
#     # bonus waypoint 1
#     if not visited[0]:
#         rel_b1 = Gps()
#         rel_b1.latitude = bonus_gps[0].latitude - robot_gps.latitude
#         rel_b1.longitude = bonus_gps[0].longitude - robot_gps.longitude
#         point_list.waypoints.append(rel_b1)
#     # bonus waypoint 2
#     if not visited[1]:
#         rel_b2 = Gps()
#         rel_b2.latitude = bonus_gps[1].latitude - robot_gps.latitude
#         rel_b2.longitude = bonus_gps[1].latitude - robot_gps.longitude
#         point_list.waypoints.append(rel_b2)
#     # bonus waypoint 3
#     if not visited[2]:
#         rel_b3 = Gps()
#         rel_b3.latitude = bonus_gps[2].latitude - robot_gps.latitude
#         rel_b3.longitude = bonus_gps[2].latitude - robot_gps.longitude
#         point_list.waypoints.append(rel_b3)
#     # final goal waypoint 
#     # (if it has been visited, the sim will stop running; don't need to check)
#     rel_goal = Gps()
#     rel_goal.latitude = goal_gps.latitude - robot_gps.latitude
#     rel_goal.longitude = goal_gps.latitude - robot_gps.longitude
#     point_list.waypoints.append(rel_goal)
#     # send list
#     pts_pub.publish(point_list)
#     # send next point as the current target position
#     loc_pub.publish(point_list.waypoints[0])