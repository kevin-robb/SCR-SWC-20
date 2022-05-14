#!/usr/bin/env python3

import rospy
from matplotlib import pyplot as plt
from swc_msgs.msg import Gps
from swc_msgs.srv import Waypoints

# list of xy-positions in meters of robot for entire runtime.
veh_x = []
veh_y = []
# 5 waypoints in meters. start, 3 bonus, goal.
wp_x = None
wp_y = None
# offset all by start pos in meters.
offset = None
# GPS -> meters conversions.
lat_to_m = 110949.14
lon_to_m = 90765.78
error_margin_lat = 1/lat_to_m
error_margin_lon = 1/lon_to_m

def interpret_waypoints(waypoints):
    global wp_x, wp_y, offset
    offset = [waypoints.waypoints[0].latitude*lat_to_m, waypoints.waypoints[0].longitude*lon_to_m]
    wp_x = [wp.latitude*lat_to_m-offset[0] for wp in waypoints.waypoints]
    wp_y = [-1 * wp.longitude*lon_to_m-offset[1] for wp in waypoints.waypoints]

def update_robot_gps(msg):
    if offset is None:
        return
    global veh_x, veh_y
    veh_x.append(msg.latitude*lat_to_m-offset[0])
    veh_y.append(-1 * msg.longitude*lon_to_m-offset[1])

    # plot landmarks.
    if wp_x is not None and wp_y is not None:
        plt.scatter(wp_y, wp_x, s=40, color="yellow", edgecolors="black")
    # plot veh pos.
    # plt.scatter(veh_x, veh_y, s=12, color="green")
    plt.scatter(veh_y[-1], veh_x[-1], s=12, color="black")

    plt.axis("equal")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Ground Truth Trajectory and Waypoints")
    plt.draw()
    plt.pause(0.00000000001)

def main():
    rospy.init_node('plotting_node')

    # subscribe to robot's current GPS position.
    # rospy.Subscriber("/sim/gps", Gps, update_robot_gps, queue_size=1)
    rospy.Subscriber("/truth/gps", Gps, update_robot_gps, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service('/sim/waypoints')
    waypoints = rospy.ServiceProxy('/sim/waypoints', Waypoints)()
    # Create GPS variables of the waypoints
    interpret_waypoints(waypoints)

    # init the plot.
    plt.ion()
    plt.show()

    # pump callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass