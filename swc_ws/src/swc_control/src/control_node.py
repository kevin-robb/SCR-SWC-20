#!/usr/bin/env python

import rospy
import time
from math import degrees
from std_msgs.msg import Float32, Bool
from swc_msgs.msg import Control
from sensor_msgs.msg import LaserScan

control_pub = None
initialized = False
# ignore obstacles farther away than clearance
clearance = 1.0
obstructed = [False]*360
# desired turn angle to target
angle_to_target = 0
# desired angle to bypass an obstacle while staying as on-target as possible
angle_bypass = 0
# bumper status (true if currently bumped)
bumped = False
turn_dir = 0 # -1 = left, 1 = right, 0 = undecided
last_time = time.time()

def get_bump_status(bump_status):
    global bumped, last_time, turn_dir
    bumped = bump_status
    last_time = time.time()
    turn_dir = 0

def choose_dir(laserscan):
    # Now decide where to go
    global angle_bypass    
    for i in range(0, 180, 5):
        if laserscan.ranges[angle_to_target - i] == 0 or laserscan.ranges[angle_to_target - i] > clearance:
            angle_bypass = angle_to_target - i
            return
        elif laserscan.ranges[angle_to_target + i] == 0 or laserscan.ranges[angle_to_target + i] > clearance:
            angle_bypass = angle_to_target + i
            return
    # if the code gets here, the bot is surrounded. just pursue the target heading and hope for the best
    angle_bypass = angle_to_target

def get_laserscan(laserscan):
    # max LIDAR range is 0.12 to 10.0 meters. gives 0 if too close or sees nothing.
    # 360 total samples, first one is straight ahead and continuing CCW.
    global obstructed
    # get min and max measured values
    #meas_min = laserscan.range_min
    #meas_max = laserscan.range_max
    for i in range(0, 360):
        if laserscan.ranges[-i] != 0 and laserscan.ranges[i] < clearance:
            obstructed[i] = True
        else:
            obstructed[i] = False
    # now choose a direction to go
    choose_dir(laserscan)

def about_to_collide():
    # range of degrees (0=straight ahead, neg=left, pos=right)
    col_range = 10
    for i in range(-col_range, col_range):
        if obstructed[i]:
            return True
    return False

def get_turn_angle(turn):
    global angle_to_target, initialized
    angle_to_target = int(degrees(turn.data)) # turn.data is in radians
    initialized = True

def timer_callback(event):
    if not initialized:
        return
    global bumped, turn_dir
    control_msg = Control()

    # modulate speed based on angle
    control_msg.speed = 5 * (1 - abs(angle_to_target)/30)**2 + 2

    # set up priority of actions
    if bumped:
        print("bumped")
        # decide which way to turn
        if turn_dir == 0:
            if angle_to_target > 0:
                turn_dir = -1
            else:
                turn_dir = 1
        # turn for a set amount of time
        if time.time() - last_time < 0.3:
            # back up and turn in the best direction
            control_msg.speed = -3
            control_msg.turn_angle = turn_dir * 25
        # go straight for a set amount of time to offset from the blocked path
        elif time.time() - last_time < 0.6:
            control_msg.speed = 3
            control_msg.turn_angle = 0
        else:
            bumped = False
            turn_dir = 0
    elif about_to_collide():
        # STOP and avoid
        control_msg.speed = 0
        print("that was close")
        bumped = True
    else:
        #print("no obstructions")
        # no obstacles in the way
        #control_msg.turn_angle = angle_to_target
        control_msg.speed = 4 * (1 - abs(angle_bypass)/30)**5 + 0.3
        control_msg.turn_angle = angle_bypass
        if control_msg.turn_angle < -90:
            control_msg.turn_angle += 180
            control_msg.speed *= -1
        elif control_msg.turn_angle > 90:
            control_msg.turn_angle -= 180
            control_msg.speed *= -1
        print("chose region", control_msg.turn_angle)
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
