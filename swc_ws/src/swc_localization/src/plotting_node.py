#!/usr/bin/env python3

import rospy
from matplotlib import pyplot as plt
from swc_msgs.msg import Gps
from swc_msgs.srv import Waypoints

def plot_waypoints(waypoints):
    # 5 waypoints: start, 3 bonus, goal.
    for wp in waypoints.waypoints:
        plt.plot(wp.latitude, wp.longitude, 'r*', markersize=15)
        # plt.axis("equal")
        # plt.draw()
        plt.pause(0.00000000001)

def plot_veh_pos(msg):
    global counter
    if counter % 10 == 0:
        plt.plot(msg.latitude, msg.longitude, 'go', markersize=10)
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)
    counter += 1

def main():
    global counter
    counter = 0

    rospy.init_node('plotting_node')

    # subscribe to robot's current GPS position.
    rospy.Subscriber("/sim/gps", Gps, plot_veh_pos, queue_size=1)

    # init the plot.
    plt.ion()
    plt.show(block=True)

    # Wait for Waypoints service and then request waypoints.
    rospy.wait_for_service('/sim/waypoints')
    waypoints = rospy.ServiceProxy('/sim/waypoints', Waypoints)()
    plot_waypoints(waypoints)

    # pump callbacks.
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass