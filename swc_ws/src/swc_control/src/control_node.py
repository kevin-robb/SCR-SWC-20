#!/usr/bin/env python

import rospy
import time
from math import degrees
from std_msgs.msg import Float32, Bool
from swc_msgs.msg import Control
from sensor_msgs.msg import LaserScan

control_pub = None
# desired turn angle to target
angle_to_target = 0
# bumper status (true if currently bumped)
bumped = False
last_time = time.time()
# laserscan status
left_in_range = False
right_in_range = False

def get_bump_status(bump_status):
    global bumped, last_time
    bumped = bump_status
    last_time = time.time()

# referenced beebotics' reactive_node.py/get_laser_val()
def get_laserscan(laserscan):
    global left_in_range, right_in_range
    # figure out which way we need to turn to avoid obstacles
    mid_index = int(len(laserscan.ranges)/2)
    # laserscan gives 0 if it sees nothing and probably 0 if it's too close, so
    # find min non-zero reading
    min_right_dist = 100000
    for scan_val in laserscan.ranges[0:mid_index]:
        if scan_val > 0 and scan_val < min_right_dist:
            min_right_dist = scan_val
    min_left_dist = 100000
    for scan_val in laserscan.ranges[mid_index:len(laserscan.ranges)-1]:
        if scan_val > 0 and scan_val < min_left_dist:
            min_left_dist = scan_val

    # check if these minimum distances are in our min range
    if min_right_dist < 0.75:
        right_in_range = True
    else:
        right_in_range = False
    if min_left_dist < 0.75:
        left_in_range = True
    else:
        right_in_range = False

def get_turn_angle(turn):
    global angle_to_target
    angle_to_target = degrees(turn.data) # turn.data is in radians

def timer_callback(event):
    global bumped
    control_msg = Control()
    # modulate speed based on angle
    control_msg.speed = 5 * (1 - abs(angle_to_target)/30)**2 + 2
    # set up priority of actions
    if bumped:
        print("bumped")
        if time.time() - last_time < 0.7:
            # back up and turn, trying to turn in the best direction
            control_msg.speed = -4
            if angle_to_target > 0:
                control_msg.turn_angle = -30
            else:
                control_msg.turn_angle = 30
        elif time.time() - last_time < 1.3:
            control_msg.speed = 5
            control_msg.turn_angle = 0
        else:
            bumped = False
    # elif right_in_range:
    #     print("right_in_range")
    #     control_msg.turn_angle = -15
    # elif left_in_range:
    #     print("left_in_range")
    #     control_msg.turn_angle = 15
    else:
        print("no obstructions")
        # no obstacles in the way
        control_msg.turn_angle = angle_to_target
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
    # subscribe to the LIDAR (updates at 10 Hz)
    scan_sub = rospy.Subscriber("/scan", LaserScan, get_laserscan, queue_size=1)
    
    # Create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
