#!/usr/bin/env python

import rospy, time
from math import atan, pi, degrees, atan2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
from swc_msgs.msg import Gps
from pure_pursuit.pure_pursuit import PurePursuit
from pure_pursuit.pp_viewer import setup_pyplot, draw_pp

turn_pub = None
# the robot's current global heading and GPS position
robot_heading = None
robot_position = None
# angle is with respect to the vertical center line, where 0 = straight up and positive is CW
# should be between -pi and pi (in radians)
desired_heading = 0

# pure pursuit path
pp = PurePursuit()
# specify whether to show the planned path plot
SHOW_PLOTS = True

# PID variables
integrator = 0
last_error = 0.0
last_time = time.time()


def get_current_heading(heading):
    global robot_heading
    #print("Got robot_heading")
    robot_heading = heading.data

def get_current_position(robot_pos):
    global robot_position
    #print("Got robot_position")
    robot_position = (robot_pos.x, robot_pos.y)

def get_desired_location(rel_goal_gps):
    global desired_heading
    lat_diff = rel_goal_gps.latitude # lat ~ y (vertical position)
    lon_diff = rel_goal_gps.longitude # lon ~ x (horizontal position)
    # calculate angle, being careful about the sign
    alpha = abs(atan(lon_diff/lat_diff))
    if lat_diff >= 0 and lon_diff >= 0:
        desired_heading = alpha
    elif lat_diff >= 0 and lon_diff < 0:
        desired_heading = -alpha
    elif lat_diff < 0 and lon_diff >= 0:
        desired_heading = pi - alpha
    elif lat_diff < 0 and lon_diff < 0:
        desired_heading = -(pi - alpha)
    # need to reverse sign because longitude increases west (along -x axis)
    desired_heading *= -1

def timer_callback(event): 
    P = 0.3 #was 0.003
    turn_angle = Float32() 

    # command heading to lookahead point
    if robot_heading is not None and robot_position is not None:
        #print("Have robot_heading and robot_position")
        heading_to_la = follow_pp_path()
        if heading_to_la is not None:
            print("-Attempting to stay on pp path.")
            turn_angle.data = (heading_to_la - robot_heading) * P
            turn_pub.publish(turn_angle)
        else:
            print("-Cannot find path. Pursuing next unvisited waypoint.")
            turn_angle.data = (desired_heading - robot_heading) * P
            # normalize to (-pi, pi)
            #turn_angle.data = (turn_angle.data + pi) % (2*pi) - pi
            turn_pub.publish(turn_angle)


    # for now let the controller worry about speed
    #print("desired heading: ", degrees(desired_heading))
    #print("current heading: ", degrees(robot_heading))

def build_pp_path(wp_path):
    global pp
    for i in range(len(wp_path.points)):
        pp.add_point(wp_path.points[i].x, wp_path.points[i].y)
    pp.display_path()

def follow_pp_path():
    global integrator, last_time, last_error

    if robot_position is None or robot_heading is None:
        # wait until localization_node brings in data to do anything
        return

    # take a snapshot of the current position so it doesn't change while this function is running
    cur_pos = (robot_position[0], robot_position[1])

    # declare the look-ahead point
    lookahead = None
    # start with a search radius of 3 meters
    radius = 3
    # look until finding the path at the increasing radius or hitting 2 meters
    while lookahead is None and radius <= 8: #was 6
        lookahead = pp.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
        radius *= 1.25
    
    # plot the planned path using pp_viewer
    if SHOW_PLOTS:
        draw_pp(cur_pos, lookahead, pp.path, xlims=[-20,20], ylims=[-5,90])

    # make sure we actually found the path
    if lookahead is not None:
        heading_to_la = degrees(atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0]))
        if heading_to_la <= 0:
            heading_to_la += 360

        #print("Current Heading: " + str(robot_heading))
        #print("Desired Heading: " + str(heading_to_la))

        #return heading_to_la

    # Test adding everything below here *******************************************************************

        delta = heading_to_la - robot_heading
        delta = (delta + 180) % 360 - 180

        # PID
        error = delta #/ 180
        time_diff = max(time.time() - last_time, 0.001)
        integrator += error * time_diff
        slope = (error - last_error) / time_diff

        P = 0.05 * error #was 0.005
        max_P = 0.25
        if abs(P) > max_P:
            # cap P and maintain sign
            P *= max_P/P
        I = 0.00001 * integrator
        D = 0.0001 * slope

        #drive_power = 1.5
        turn_power = P + I + D

        last_error = error
        last_time = time.time()

        return -turn_power #TODO added this minus sign on a hunch. it might be wrong.

        # # make the motors command
        # motor_msg = motors()
        # motor_msg.left = drive_power - turn_power
        # motor_msg.right = drive_power + turn_power
        
        # command_pub.publish(motor_msg)

    #********************************************************************************************************

def main():
    global turn_pub

    # Initalize our node in ROS
    rospy.init_node('localization_node')

    # Create publisher for turn angle
    turn_pub = rospy.Publisher("/swc/turn_cmd", Float32, queue_size=1)

    # Create subscribers to get the desired relative location, current location, and current heading
    goal_sub = rospy.Subscriber("/swc/goal", Gps, get_desired_location, queue_size=1)
    pos_sub = rospy.Subscriber("/swc/current_position", Point32, get_current_position, queue_size=1)
    hdg_sub = rospy.Subscriber("/swc/current_heading", Float32, get_current_heading, queue_size=1)
    # subscribe to the set of waypoints published by localization_node for pure pursuit
    path_sub = rospy.Subscriber("/swc/wp_path", PointCloud, build_pp_path, queue_size=1)

    # Create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    # pump callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
