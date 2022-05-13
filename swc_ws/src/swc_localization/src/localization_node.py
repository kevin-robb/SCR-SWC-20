#!/usr/bin/env python3

import rospy
from tf import transformations
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from swc_msgs.msg import Gps
from swc_msgs.srv import Waypoints

loc_pub = None
hdg_pub = None
dist_pub = None
# robot's current GPS location
robot_gps = Gps()
# waypoints
start_gps = Gps()
bonus_gps = []
goal_gps = Gps()
wp_interpreted = False
# keep track of which waypoints have been visited
visited = [False, False, False]
lat_to_m = 110949.14
lon_to_m = 90765.78
error_margin_lat = 1/lat_to_m
error_margin_lon = 1/lon_to_m

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
    if abs(point_gps.latitude - robot_gps.latitude) < error_margin_lat and abs(point_gps.longitude - robot_gps.longitude) < error_margin_lon:
        return True
    else:
        return False

def update_robot_gps(gps_reading):
    global robot_gps, visited
    robot_gps = gps_reading
    # check all the bonus waypoints to see if visited.
    # make sure the waypoints have been interpreted first.
    if wp_interpreted:
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
        pub_next_pt()

def pub_next_pt():
    # Find and publish relative location of next non-visited point.
    rel = Gps()
    # bonus waypoint 1
    if not visited[0]:
        #print("going for point 1")
        rel.latitude = bonus_gps[0].latitude - robot_gps.latitude
        rel.longitude = bonus_gps[0].longitude - robot_gps.longitude
        loc_pub.publish(rel)
        pub_dist_to_next_pt(bonus_gps[0])
    # bonus waypoint 2
    elif not visited[1]:
        #print("going for point 2")
        rel.latitude = bonus_gps[1].latitude - robot_gps.latitude
        rel.longitude = bonus_gps[1].longitude - robot_gps.longitude
        loc_pub.publish(rel)
        pub_dist_to_next_pt(bonus_gps[1])
    # bonus waypoint 3
    elif not visited[2]:
        #print("going for point 3")
        rel.latitude = bonus_gps[2].latitude - robot_gps.latitude
        rel.longitude = bonus_gps[2].longitude - robot_gps.longitude
        loc_pub.publish(rel)
        pub_dist_to_next_pt(bonus_gps[2])
    # final goal waypoint 
    else:
        #print("going for final goal")
        rel.latitude = goal_gps.latitude - robot_gps.latitude
        rel.longitude = goal_gps.longitude - robot_gps.longitude
        loc_pub.publish(rel)
        pub_dist_to_next_pt(goal_gps)

def pub_dist_to_next_pt(point_gps):
    dist = Float32()
    # convert dist from GPS to meters
    lat_diff = (point_gps.latitude - robot_gps.latitude) * lat_to_m
    lon_diff = (point_gps.longitude - robot_gps.longitude) * lon_to_m
    dist.data = (lat_diff**2 + lon_diff**2)**(1/2)
    #print("dist to target:", dist.data)
    dist_pub.publish(dist)
    # used in control_node to allow sharp turns when near the goal to prevent missing it

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
    current_heading.data = -yaw_rads
    hdg_pub.publish(current_heading)

def main():
    global loc_pub, hdg_pub, dist_pub

    # Initalize our node in ROS
    rospy.init_node('localization_node')

    # publish target point location relative to robot's current position
    loc_pub = rospy.Publisher("/swc/goal", Gps, queue_size=1)
    # publish robot's current heading
    hdg_pub = rospy.Publisher("/swc/current_heading", Float32, queue_size=1)
    # publish distance to current target waypoint
    dist_pub = rospy.Publisher("/swc/dist", Float32, queue_size=1)

    # subscribe to robot's current GPS position and IMU data
    rospy.Subscriber("/sim/gps", Gps, update_robot_gps, queue_size=1)
    rospy.Subscriber("/sim/imu", Imu, update_heading, queue_size=1)

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