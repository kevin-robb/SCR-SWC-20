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
# desired turn angle to target
angle_to_target = 0
# desired angle to bypass an obstacle while staying as on-target as possible
angle_bypass = 0
# bumper status (true if currently bumped)
bumped = False
turn_dir = 0 # -1 = left, 1 = right, 0 = undecided
last_time = time.time()
# keeps track of closest obstacle in each region
obs = [0,0,0,0,0,0,0,0] #obs[0] is front-left region, CCW in 45 degree regions

def get_bump_status(bump_status):
    global bumped, last_time, turn_dir
    bumped = bump_status
    last_time = time.time()
    turn_dir = 0

# def reg_to_hdg(reg):
#     # convert region index to degrees
#     return (((reg + 16) % 8) - 4) * -45

def choose_dir(laserscan):
    # Now decide where to go
    global angle_bypass
    # target_reg = int((angle_to_target + 180.0) / -45.0) #index of region matching obs
    # # if the target region is totally unoccupied, go there
    # if obs[target_reg] == 0:
    #     angle_bypass = reg_to_hdg(target_reg)
    #     return
    
    for i in range(0, 180, 5):
        if laserscan.ranges[angle_to_target - i] == 0 or laserscan.ranges[angle_to_target - i] > clearance:
            angle_bypass = angle_to_target - i
            return
        elif laserscan.ranges[angle_to_target + i] == 0 or laserscan.ranges[angle_to_target + i] > clearance:
            angle_bypass = angle_to_target + i
            return
    # if the code gets here, the bot is surrounded. just pursue the target heading and hope for the best
    angle_bypass = angle_to_target

    # # choose which region to go to depending on obstructions and target heading
    # #global angle_to_target, obs
    # # angle_to_target is in the range -180 to 180 degrees (CW, 0 is front)
    # target_reg = int((angle_to_target + 180.0) / -45.0) #index of region matching obs
    # # now find the closest unoccupied region to the target region
    # if obs[target_reg] == 0:
    #     return reg_to_hdg(target_reg)
    # # max_reg = target_reg
    # # max_val = obs[target_reg]
    # # for i in range(0,4):
    # #     # check for empty region
    # #     if obs[target_reg - i] == 0:
    # #         return reg_to_hdg(target_reg - i)
    # #     elif obs[target_reg + i] == 0:
    # #         return reg_to_hdg(target_reg + i)
    # #     # find best non-empty region (largest distance away)
    # #     if obs[target_reg - i] > max_val:
    # #         max_reg = target_reg - i
    # #         max_val = obs[target_reg - i]
    # #     if obs[target_reg + i] > max_val:
    # #         max_reg = target_reg + i
    # #         max_val = obs[target_reg + i]
    # # return reg_to_hdg(max_reg)


def get_laserscan(laserscan):
    # max LIDAR range is 0.12 to 10.0 meters. gives 0 if too close or sees nothing.
    # 360 total samples, first one is straight ahead and continuing CCW.
    global obs
    # get min and max measured values
    meas_min = laserscan.range_min
    #meas_max = laserscan.range_max
    # create 8 directional thing for whether obstacles are present in each region
    for reg in range(0, 8):
        min_in_range = 100
        for i in range(0, 45*(reg+1)):
            if laserscan.ranges[i] >= meas_min and laserscan.ranges[i] < min_in_range and laserscan.ranges[i] < clearance:
                min_in_range = laserscan.ranges[i]
        obs[reg] = min_in_range if min_in_range < 100 else 0
    # now choose a direction to go
    choose_dir(laserscan)

    # max_reg = target_reg
    # max_val = obs[target_reg]
    # for i in range(0,4):
    #     # check for empty region
    #     if obs[target_reg - i] == 0:
    #         return reg_to_hdg(target_reg - i)
    #     elif obs[target_reg + i] == 0:
    #         return reg_to_hdg(target_reg + i)
    #     # find best non-empty region (largest distance away)
    #     if obs[target_reg - i] > max_val:
    #         max_reg = target_reg - i
    #         max_val = obs[target_reg - i]
    #     if obs[target_reg + i] > max_val:
    #         max_reg = target_reg + i
    #         max_val = obs[target_reg + i]
    # return reg_to_hdg(max_reg)


    ##*********************************************************************************************old
    # global obs_ahead, obs_left, obs_right
    # # set clearances for whether obstacles will be considered obstructions. meters
    # front_clearance = 1.0
    # side_clearance = 0.15
    # # get min and max measured values
    # meas_min = laserscan.range_min
    # #meas_max = laserscan.range_max
    # max_deg = 20 # degrees of each section
    # # first check if there is an obstacle straight ahead
    # #obs_ahead = laserscan.ranges[0] >= meas_min and laserscan.ranges[0] < front_clearance
    # for i in range(-max_deg/2, max_deg/2):
    #     if laserscan.ranges[i] >= meas_min and laserscan.ranges[i] < front_clearance:
    #         obs_ahead = True
    #         break
    #     else:
    #         obs_ahead = False
    # # next check if there is an obstacle on either side
    # # check left (positive degrees starting from straight ahead)
    # for i in range(max_deg/2, 3*max_deg/2):
    #     if laserscan.ranges[i] >= meas_min and laserscan.ranges[i] < side_clearance:
    #         obs_left = True
    #         break
    #     else:
    #         obs_left = False
    # # check right (negative degrees starting from straight ahead)
    # for i in range(max_deg/2, 3*max_deg/2):
    #     if laserscan.ranges[360 - i] >= meas_min and laserscan.ranges[360 - i] < side_clearance:
    #         obs_right = True
    #         break
    #     else:
    #         obs_right = False
    ##**********************************************************************************************\old


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
    # elif obs_ahead:
    #     # obstacle straight ahead. check sides before dodging
    #     # slow down and turn to prevent hitting the obstacle
    #     control_msg.speed = 0.5
    #     if obs_left and not obs_right:
    #         # turn right
    #         print("dodge right")
    #         control_msg.turn_angle = -15
    #     elif obs_right and not obs_left:
    #         # turn left
    #         print("dodge left")
    #         control_msg.turn_angle = 15
    #     elif not obs_left and not obs_right:
    #         print("dodge")
    #         # just turn right by default
    #         control_msg.turn_angle = 30
    #     else:
    #         # obstacle ahead and both sides, so TODO back up until one side is clear
    #         print("trapped")
    #         control_msg.speed = -1
    #         control_msg.turn_angle = -20
    # elif obs_left and obs_right:
    #     # we are between two obstacles somehow. keep going straight
    #     print("between obstacles")
    #     control_msg.turn_angle = angle_to_target
    # elif obs_left:
    #     # obstacle on left but not ahead. little dodge
    #     print("little dodge left")
    #     control_msg.turn_angle = 15
    # elif obs_right:
    #     # obstacle on right but not ahead. little dodge
    #     print("little dodge right")
    #     control_msg.turn_angle = -15
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
