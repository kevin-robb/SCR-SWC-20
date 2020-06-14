#!/usr/bin/env python

import rospy
import time
from math import degrees
from std_msgs.msg import Float32, Bool
from swc_msgs.msg import Control
from sensor_msgs.msg import LaserScan

control_pub = None
initialized = False
# desired turn angle to target (forward = 0, CW is positive)
angle_to_target = 0
dist_to_target = 100
# bumper status (true if currently bumped)
bumped = False
# heading to target modified to bypass obstacles
command_heading = 0

def get_bump_status(bump_status):
    global bumped
    bumped = bump_status
    print("bumped", bumped)

def get_turn_angle(turn):
    # receive desired heading directly to target
    global angle_to_target, initialized
    angle_to_target = int(degrees(turn.data)) # turn.data is in radians
    initialized = True

def get_bypass_angle(bypass_hdg):
    # receive commanded heading modification to avoid collisions (from scan_node)
    global command_heading
    if bypass_hdg.data == 0:
        command_heading = angle_to_target - bypass_hdg.data
    else:
        command_heading = 180 - bypass_hdg.data #TODO make sure this is right with the coord system

def get_dist_to_target(dist):
    global dist_to_target
    dist_to_target = dist.data

def timer_callback(event):
    if not initialized:
        return
    global bumped
    control_msg = Control()

    # check bumpers first.
    if bumped:
        print("action: bumped")
        # decide which way to turn.
        turn_dir = 0
        if command_heading > 0:
            turn_dir = -1
        else:
            turn_dir = 1
        # back up a bit.
        backup_time = 0.4
        last_time = time.time()
        while time.time() - last_time < backup_time:
            control_msg.speed = -2
            control_msg.turn_angle = 0
            control_pub.publish(control_msg)
        # turn according to turn_dir.
        turn_time = 0.6
        last_time = time.time()
        while time.time() - last_time < turn_time:
            control_msg.speed = 2
            control_msg.turn_angle = turn_dir * 25
            control_pub.publish(control_msg)
        bumped = False
    else:
        # pursue angle to next waypoint.
        # modulate speed based on angle.
        control_msg.speed = 5 * (1 - abs(command_heading)/30)**5 + 1.0 # was + 0.5
        # reduce oscillations with a P-controller
        P = 0.3
        # if we are very close to a waypoint, don't clamp the angle as much (prevent missing)
        if dist_to_target < 6:
            P = 0.6
        control_msg.turn_angle = command_heading * P
        # correct for really big turns (unlikely)
        if control_msg.turn_angle < -90:
            control_msg.turn_angle += 180
            control_msg.speed *= -1
        elif control_msg.turn_angle > 90:
            control_msg.turn_angle -= 180
            control_msg.speed *= -1
        print("control heading: ", control_msg.turn_angle)
        control_pub.publish(control_msg)

def main():
    global control_pub

    # Initalize our node in ROS
    rospy.init_node('control_node')

    # publish the command messages
    control_pub = rospy.Publisher("/sim/control", Control, queue_size=1)

    # subscribe to nav's commanded turn angle
    turn_sub = rospy.Subscriber("/swc/turn_cmd", Float32, get_turn_angle, queue_size=1)
    # subscribe to the front bump sensor
    bump_sub = rospy.Subscriber("/sim/bumper", Bool, get_bump_status, queue_size=1)
    # subscribe to the scan_node's commanded angle to bypass obstructions
    bypass_sub = rospy.Subscriber("/swc/bypass", Float32, get_bypass_angle, queue_size=1)
    # subscribe to the distance to the current target waypoint
    dist_sub = rospy.Subscriber("/swc/dist", Float32, get_dist_to_target, queue_size=1)
    
    # Create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
