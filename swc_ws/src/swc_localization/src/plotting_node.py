#!/usr/bin/env python3

import rospy
from matplotlib import pyplot as plt
from swc_msgs.msg import Gps
from swc_msgs.srv import Waypoints
import atexit

# list of xy-positions in meters of robot for entire runtime.
veh_x = []
veh_y = []
# 5 waypoints in meters. start, 3 bonus, goal.
wp_x = []
wp_y = []
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
    wp_y = [wp.longitude*lon_to_m-offset[1] for wp in waypoints.waypoints]

def update_robot_gps(msg):
    if offset is None:
        return
    global veh_x, veh_y
    veh_x.append(msg.latitude*lat_to_m-offset[0])
    veh_y.append(msg.longitude*lon_to_m-offset[1])

def make_plot():
    plt.figure(figsize=(8,7))
    plt.grid(True)
    plt.scatter(wp_x, wp_y, s=40, color="yellow", edgecolors="black")
    plt.scatter(veh_x, veh_y, s=12, color="red")

    # other plot formatting.
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("SWC Trajectory")
    plt.tight_layout()
    # save to a file in plots/ directory.
    # plt.savefig("./plots/"+FILE_ID+".png", format='png')
    # show the plot.
    plt.show()


def main():
    rospy.init_node('plotting_node')

    # when the node exits, make the plot.
    atexit.register(make_plot)

    # subscribe to robot's current GPS position.
    rospy.Subscriber("/sim/gps", Gps, update_robot_gps, queue_size=1)

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