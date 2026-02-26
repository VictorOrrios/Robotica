#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot

def eight_shape_time_based(robot: Robot, speed: float, radius: float):
    robot.lock_odometry.acquire()
    print("Odom values at Start. X= ", robot.x.value, ", Y= ", robot.y.value, ", TH= ", robot.th.value)
    robot.lock_odometry.release()

    # PART 1:
    robot.setSpeed(0,np.deg2rad(-45))
    time.sleep(2.0)
    """robot.lock_odometry.acquire()
    print("Odom values at Part 1. X= ", robot.x.value, ", Y= ", robot.y.value, ", TH= ", robot.th.value)
    robot.lock_odometry.release()

    # PART 2:
    robot.setSpeed(speed,speed/radius)
    time.sleep(np.pi*radius/speed)
    robot.lock_odometry.acquire()
    print("Odom values at Part 2. X= ", robot.x.value, ", Y= ", robot.y.value, ", TH= ", robot.th.value)
    robot.lock_odometry.release()

    # PART 3:
    robot.setSpeed(speed,-speed/radius)
    time.sleep(2*np.pi*radius/speed)
    robot.lock_odometry.acquire()
    print("Odom values at Part 3. X= ", robot.x.value, ", Y= ", robot.y.value, ", TH= ", robot.th.value)
    robot.lock_odometry.release()

    # PART 4:
    robot.setSpeed(speed,speed/radius)
    time.sleep(np.pi*radius/speed)
    robot.lock_odometry.acquire()
    print("Odom values at Part 4. X= ", robot.x.value, ", Y= ", robot.y.value, ", TH= ", robot.th.value)
    robot.lock_odometry.release()

    # ...

    robot.lock_odometry.acquire()
    print("Odom values at main at the END. X= ", robot.x.value, ", Y= ", robot.y.value, ", TH= ", robot.th.value)
    robot.lock_odometry.release()"""
    
def poll_odometry_until(robot: Robot, values: [float], tolerance_left: [float], tolerance_right: [float], poll_interval_seconds: float):
    while True:
        x, y, th = robot.readOdometry()
        # threshold is [x, y, th IN RADIANS]
        # TODO, tune error thresholds based on expected error
        x_met, y_met, th_met = False, False, False
    
        x_met = values[0] - tolerance_left[0] <= x <= values[0] + tolerance_right[0] # x - something (for tolerance)
        y_met = values[1] - tolerance_left[1] <= y <= values[1] + tolerance_right[1]
        th_met = values[2] - tolerance_left[2] <= th <= values[2] + tolerance_right[2]

        if x_met and y_met and th_met:
            print("Reached threshold. Current odom: x=", x, ", y=", y, ", th=", th)
            break
        time.sleep(poll_interval_seconds)



def eight_shape_odometry_based(robot: Robot, speed: float, radius: float, poll_interval_seconds: float):
    robot.lock_odometry.acquire()
    print("Odom values at Start. X= ", robot.x.value, ", Y= ", robot.y.value, ", TH= ", robot.th.value)
    robot.lock_odometry.release()

    tolerances_1 = [0.05, 0.05, np.deg2rad(1)]
    # PART 1 (rotate in place):
    robot.setSpeed(0,np.deg2rad(-45))
    poll_odometry_until(robot, (0.0, 0.0, np.deg2rad(-90)), tolerances_1, tolerances_1, poll_interval_seconds)
    """robot.lock_odometry.acquire()
    print("Odom values at Part 1. X= ", robot.x.value, ", Y= ", robot.y.value, ", TH= ", robot.th.value)
    robot.lock_odometry.release()"""
    print("Part 1 complete")
    
    tolerances_2_left = [0.02, 0.02, np.deg2rad(2)]
    tolerances_2_right = [0.02, 0.02, np.deg2rad(2)]
    # PART 2 (first half of the first loop):
    robot.setSpeed(speed,speed/radius)
    poll_odometry_until(robot, (2*radius, 0.0, np.deg2rad(90)), tolerances_2_left, tolerances_2_right, poll_interval_seconds)
    print("Part 2 complete")

    tolerances_3_left = [0.02, 0.02, np.deg2rad(2)]
    tolerances_3_right = [0.02, 0.02, np.deg2rad(0)]
    # PART 3 (first half of the second loop):
    robot.setSpeed(speed,-speed/radius)
    poll_odometry_until(robot, (4*radius, 0.0, np.deg2rad(-90)), tolerances_3_left, tolerances_3_right, poll_interval_seconds)
    print("Part 3 complete")
    
    tolerances_4_left = [0.02, 0.02, np.deg2rad(2)]
    tolerances_4_right = [0.02, 0.02, np.deg2rad(1)]
    # PART 4 (second half of the second loop):
    # robot.setSpeed(speed,-speed/radius) # No need, we just keep moving with the same speed
    poll_odometry_until(robot, (2*radius, 0.0, np.deg2rad(90)), tolerances_4_left, tolerances_4_right, poll_interval_seconds)
    print("Part 4 complete")

    # PART 5 (second half of the first loop):
    tolerances_5_left = [0.02, 0.02, np.deg2rad(1)]
    tolerances_5_right = [0.02, 0.02, np.deg2rad(1)]
    robot.setSpeed(speed,speed/radius)
    poll_odometry_until(robot, (0.0, 0.0, np.deg2rad(-90)), tolerances_5_left, tolerances_5_right, poll_interval_seconds)
    print("Part 5 complete")
    
def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        x_ini=0.0
        y_ini=0.0
        th_ini = 0.0*np.pi/180.0

        # TODO, set odometry update interval as constructor parameter
        LEGO_WHEEL_RADIUS = 0.028 # Sim: 0.04, Lego: 0.028
        LEGO_AXIS_LENGTH = 0.12 # 15 studs, distance between center of 2 studs = 8mm -> NLego: 0.2
        LEGO_ODOMETRY_UPDATE_PERIOD = 0.035 # seconds -> 0.05 for lego
        robot = Robot([x_ini, y_ini, th_ini], LEGO_WHEEL_RADIUS, LEGO_AXIS_LENGTH, LEGO_ODOMETRY_UPDATE_PERIOD)

        print("X value at the beginning from main =", robot.x.value)
        print("Y value at the beginning from main =", robot.y.value)
        print("TH value at the beginning from main =", robot.th.value)

        # 1. launch updateOdometry Process()
        robot.startOdometry()
        time.sleep(2)
        
        speed = 0.1 # Sim -> 0.2
        radius = 0.2 # Sim -> 0.8
        
        # eight_shape_time_based(robot, speed, radius)
        eight_shape_odometry_based(robot, speed, radius, poll_interval_seconds=0.01)
        
        robot.stopOdometry()

        # 2. perform trajectory

        # Demo code (move 4 seconds at 0.1 m/s)
        
        #print("Start :",  time.ctime(), " : converts a time in seconds since the epoch to a string in local times")
        
        #robot.setSpeed(0.1,0)
        #robot.setSpeed(0,np.deg2rad(45))

        #tIni = time.perf_counter()
        #total_time = 0.0
        #while (total_time < 4.0):

        #     [v,w]=robot.readSpeed()
        #     print("Moving at v", v, " m/s, and w=",  w, "rad/s")

        #     [x,y,th]=robot.readOdometry()
        #     print("Robot placed at x=", x, "y=", y, "th=", th)
        #     time.sleep(0.05)

        #     tEnd = time.perf_counter()
        #     total_time = (tEnd-tIni)
        
        #time.sleep(3)
        
        
        #print("End")

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.

    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)



