#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

bypass_pub = None

# data from LIDAR (list with 360 entries, distance to obstacle if any).
    # max LIDAR range is 0.12 to 10.0 meters. gives 0 if too close or sees nothing.
    # 360 total samples, first one is straight ahead and continuing CCW in a full circle.
laserscan = None
# min (non-zero) measured value
meas_min = 0.01
# ignore obstacles farther away than clearance
clearance = 5

def obstacle_in_range(index):
    return laserscan.ranges[index] >= meas_min and laserscan.ranges[index] < clearance

def find_obstacles():
    print("-looking for obstacles")
    # first check if there are any obstacles within range of the robot (on any side)
    no_obstacles_within_range = True
    for i in range(0, 360):
        if obstacle_in_range(i):
            no_obstacles_within_range = False
            break
    if no_obstacles_within_range:
        print("--no obstacles found")
        return None

    # find points of importance for each obstacle within range:
        # 0 - rightmost point the robot can see
        # 1 - nearest point of obstacle to robot
        # 2 - leftmost point the robot can see
    obstacles = [] # each entry will be a tuple of laserscan indexes (right, near, left)
    obs = [0, 0, 0] # temp storage of one entry
    closest_dist = clearance # temp for finding nearest point on each obstacle
    
    # decide the starting position for obstacle search
    start_pos = 0
    if obstacle_in_range(0):
        # there is an obstacle straight ahead
        # look to the right until finding the right side of this obstacle, and proceed as normal
        start_pos = 0
        for i in range(0, -360):
            if not obstacle_in_range(i):
                start_pos = i
                break
    else:
        # there is nothing straight ahead
        start_pos = 0

    # search the entire LIDAR range and save indexes of relevant points for each obstacle
    all_clear = True
    for i in range(start_pos, (start_pos + 360) % 360):
        print("--loop is running to search for obstacles")
        if obstacle_in_range(i) and all_clear:
            # new obstacle (right side of it)
            obs[0] = i
            closest_dist = laserscan.ranges[i]
            obs[1] = i
        elif obstacle_in_range(i) and not all_clear:
            # same obstacle we already started building
            # check if this is closer than the current nearest point
            if closest_dist > laserscan.ranges[i]:
                closest_dist = laserscan.ranges[i]
                obs[1] = i
        elif not obstacle_in_range(i) and not all_clear:
            # just passed the left side of an obstacle
            obs[2] = i - 1
            # we now have all the desired data about that obstacle, 
            # so add it to the list and reset obs to prep for the next one
            obstacles.append((obs[0], obs[1], obs[2]))
            obs = [0, 0, 0]
            closest_dist = clearance
            all_clear = True
            print("--found an obstacle")

    return obstacles

def determine_obstruction():
    print("determining obstructions")
    # construct a local map of all obstacles to help avoid the forward obstruction.
    obstacles = find_obstacles()
    # if there are no obstacles near the robot, do not modify the current heading
    if obstacles is None or len(obstacles) == 0:
        print("-no obstacles")
        return 0

    # we will say that anything within 20 degrees to either 
    # side of the robot's current trajectory is a potential obstruction.
    obs_ahead = False
    for i in range(-20, 20):
        if obstacle_in_range(i):
            obs_ahead = True
    # skip all the computation if there is no obstruction ahead.
    if not obs_ahead:
        print("-safe to continue straight")
        # command the current heading (no alteration necessary)
        return 0

    # if there is a forward obstruction (if we got here, there is), it will be the first entry.
    front_obs = obstacles[0]
    # determing the best heading to command to bypass the obstacle.
    bypass_hdg = 0
    print("-calculating bypass_heading")
    if len(obstacles) >= 2:
        print("--multiple obstacles around")
        # if there are at least 2 obstacles, we can define an all_clear region on each side.
        left_clear_range = (obstacles[0][2], obstacles[1][0])
        left_gap = left_clear_range[1] - left_clear_range[0]
        right_clear_range = (obstacles[-1][0], obstacles[0][2])
        right_gap = right_clear_range[1] - right_clear_range[0]
        # check which side has more clearance, and command a heading.
        if left_gap > right_gap:
            if left_gap < 50:
                bypass_hdg = (left_clear_range[1] + left_clear_range[0])/2
            else:
                bypass_hdg = left_clear_range[0] + 25
        else:
            if right_gap < 50:
                bypass_hdg = (right_clear_range[1] + right_clear_range[0])/2
            else:
                bypass_hdg = right_clear_range[1] - 25
    else:
        print("--single obstacle ahead")
        # there is only one obstacle, straight ahead. 
        # command a heading to bypass it on the side requiring less divergence from current path.
        if abs(front_obs[0]) < abs(front_obs[2]):
            bypass_hdg = obstacles[0][0] + 20
        else:
            bypass_hdg = obstacles[0][2] - 20
    # be more agressive the closer the forward obstacle is
    dist_to_obs = front_obs[1]
    # scale distance as a percent of clearance
    dist_to_obs_scaled = dist_to_obs / clearance
    print("-sending bypass_heading")
    return bypass_hdg / dist_to_obs_scaled

def receive_laserscan(ls):
    global laserscan, meas_min
    laserscan = ls
    meas_min = laserscan.range_min

    # determine if the robot's current path is obstructed, and if so, command a heading to avoid collision.
    # this is called here so the laserscan values do not change while it is running
    #cmd_hdg = determine_obstruction()
    cmd_hdg = simple_avoidance()

    bypass_msg = Float32()
    bypass_msg.data = cmd_hdg
    bypass_pub.publish()

def simple_avoidance():
    if obstacle_in_range(0):
        # there is an obstacle straight ahead. avoid it by turning.
        print("obstacle detected ahead")
        # find position of left and right clearance
        l_clr = 0
        r_clr = 0
        for i in range(355, 180):
            if not obstacle_in_range(i):
                print("-right clearance at degrees: ", i)
                l_clr = i
                break
        for i in range(0, 180):
            if not obstacle_in_range(i):
                print("-left clearance at degrees: ", i)
                r_clr = i
                break
        if l_clr == 0 and r_clr == 0:
            print("-no clearance found")
            return 0
        # command a heading to the easier side
        if l_clr < r_clr and l_clr != 0:
            # pass on the left
            return l_clr + 15
        else:
            # pass on the right
            return r_clr - 15
        return 0 #TODO this line should never be reached
    else:
        # the way ahead is clear. continue straight.
        return 0


def main():
    global bypass_pub

    # Initalize our node in ROS
    rospy.init_node('scan_node')

    # publish the commanded heading to bypass a forward obstruction.
    bypass_pub = rospy.Publisher("/swc/bypass", Float32, queue_size=1)

    # subscribe to the LIDAR (updates at 10 Hz)
    scan_sub = rospy.Subscriber("/scan", LaserScan, receive_laserscan, queue_size=1)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
