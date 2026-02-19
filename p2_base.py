#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot


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

        robot = Robot([x_ini, y_ini, th_ini])

        print("X value at the beginning from main =", robot.x.value)
        print("Y value at the beginning from main =", robot.y.value)
        print("TH value at the beginning from main =", robot.th.value)

        # 1. launch updateOdometry Process()
        robot.startOdometry()
        time.sleep(2)

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

        robot.lock_odometry.acquire()
        print("Odom values at Start. X= ", robot.x.value, ", Y= ", robot.y.value, ", TH= ", robot.th.value)
        robot.lock_odometry.release()
        
        speed = 0.2
        radius = 0.8


        # PART 1:
        robot.setSpeed(0,np.deg2rad(-45))
        time.sleep(2)
        robot.lock_odometry.acquire()
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

        # PART 2:
        robot.setSpeed(speed,speed/radius)
        time.sleep(np.pi*radius/speed)
        robot.lock_odometry.acquire()
        print("Odom values at Part 4. X= ", robot.x.value, ", Y= ", robot.y.value, ", TH= ", robot.th.value)
        robot.lock_odometry.release()

        # ...

        robot.lock_odometry.acquire()
        print("Odom values at main at the END. X= ", robot.x.value, ", Y= ", robot.y.value, ", TH= ", robot.th.value)
        robot.lock_odometry.release()

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()


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



