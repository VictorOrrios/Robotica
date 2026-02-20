#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import brickpi3 # import the BrickPi3 drivers
# import CoppeliaAPI.csimBrickpi as brickpi3 # import CopelliaSim api
import time     # import the time library for the sleep function
import sys
import numpy as np
import csv
import matplotlib
matplotlib.use("TkAgg")

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0], R: float=0.04, L: float=0.20, P: float=0.15):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """
        
        ######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # # Robot construction parameters (CHANGE FOR YOUR OWN)
        # self.R = 0.04
        # self.L = 0.20

        # Coppelia Robot construction parameters
        self.R = R
        self.L = L

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # # Configure sensors, for example a touch sensor or a gyro
        # self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)
        # self.BP.set_sensor_type(self.BP.PORT_4, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
        
        # # reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B))
        self.BP.offset_motor_encoder(self.BP.PORT_C, self.BP.get_motor_encoder(self.BP.PORT_C))
        
        ##################################################
        # odometry shared memory values
        self.x = Value('d',init_position[0])
        self.y = Value('d',init_position[1])
        self.th = Value('d',init_position[2])
        self.finished = Value('b',1) # boolean to show if odometry updates are finished
        
        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        #self.lock_odometry.acquire()
        #print('hello world', i)
        #self.lock_odometry.release()
        ##################################################
        
        # Last two encoder measurements. Used to compute the speed of the wheels. in DEGREES
        # Make sure your code is implemented so that they are obtained with period self.P 
        self.prev_encoder_d = Value('d',0.0)
        self.prev_encoder_i = Value('d',0.0)
        self.cur_encoder_d = Value('d',0.0)
        self.cur_encoder_i = Value('d',0.0)
        self.lock_encoder = Lock()
        
        # odometry update period
        self.P = P
        

    def normaliza_rad(self, angle_in): # Lleva al rango -pi .. +pi
        angle_out = (angle_in + np.pi) % (2 * np.pi) - np.pi
        # print("Processing the angle (in radians) ", angle_in, ". We obtain ", angle_out, " (in radians)")
        return angle_out

    def setSpeed(self, v,w):
        # print("setting speed to v=", v, ", w=", w)
        # v in [m/s] and w in [rad/s]
        
        speedRad_right = v/self.R + w*self.L/(2*self.R)
        speedRad_left = v/self.R - w*self.L/(2*self.R)
        speedDPS_right = np.rad2deg(speedRad_right)
        speedDPS_left = np.rad2deg(speedRad_left)
        self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_right)
        self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_left)


    def readSpeed(self):
        # Here, do NOT udpate self.cur_encoder_d.value or self.prev_encoder_d.value
        # Instead, use these values to compute wheel speeds, and to transform 
        # wheel speeds into robot linear and angular speeds.
        print("Read speed")

        # Read encoder values in mutual exclusion (minimize exclusive access overhead)
        self.lock_encoder.acquire()
        curr_enc_d, prev_enc_d = self.cur_encoder_d.value, self.prev_encoder_d.value
        curr_enc_i, prev_enc_i = self.cur_encoder_i.value, self.prev_encoder_i.value
        self.lock_encoder.release()
        
        wd = (curr_enc_d - prev_enc_d) / self.P
        wi = (curr_enc_i - prev_enc_i) / self.P

        print("wd: ", wd, ", wi: ", wi)

        wd = np.deg2rad(wd)
        wi = np.deg2rad(wi)
        v = (wd+wi)*self.R/2
        w = (wd-wi)*self.R/self.L
        return v,w

    def readOdometry(self):
        # Here, do NOT udpate self.x.value, self.y.value or self.th.value
        """ Returns current value of odometry estimation """
        self.lock_odometry.acquire()
        # Read values in mutual exclusion
        x, y, th = self.x.value, self.y.value, self.th.value
        self.lock_odometry.release()
        return x,y,th        

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()
        print("PID: ", self.p.pid)

    def updateOdometry(self):
        try:
            """ To be filled ...  """
            # Wait until the gyro sensor is ready
            
            # Initialize the CSV file to write the logs
            # print("Start odometry:",  time.ctime(), " : converts a time in seconds since the epoch to a string in local times")
            
            csvfile = open('mi_prueba.csv', 'w', newline='') # Modify the name as desired
            header = ['t', 'x', 'y', 'th']  # Include more fields if you want
            csvwriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            csvwriter.writerow(header)

            tIniCSV = time.perf_counter() # To measure times with high precission. The absolute value is no useful. 
            #                               Instead, use it for computing elapsed times. 
            
            # Initialize a figure for plotting
            #current_fig = plt.figure()
            
            while not self.finished.value:
                # High precission but used only to compute elapsed times (no use for absolute time)
                tIni = time.perf_counter()

                # compute updates

                ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
            
                try:
                    # READ ENCODERS.
                    print("Reading encoder values ....")

                    # Update the last stored econder values

                    # Read the motor encoder values NOT in mutual exclusion (to avoid overhead)
                    motor_encoder_d_value = self.BP.get_motor_encoder(self.BP.PORT_C)
                    motor_encoder_i_value = self.BP.get_motor_encoder(self.BP.PORT_B)
                    
                    # Read/write encoder values in mutual exclusion
                    self.lock_encoder.acquire()
                    self.prev_encoder_d.value = self.cur_encoder_d.value
                    self.prev_encoder_i.value = self.cur_encoder_i.value

                    [self.cur_encoder_d.value, self.cur_encoder_i.value] = [motor_encoder_d_value,
                        motor_encoder_i_value]
                    self.lock_encoder.release()
                    
                    # If you read more sensors, you may need to wait a bit between reading
                    #time.sleep(0.02)
                    
                    # TODO, read them in mutual exclusion
                    # print("Encoder Right increased in decrees in : ", self.cur_encoder_d.value - self.prev_encoder_d.value)
                    # print("Encoder Left increased in decrees in : ", self.cur_encoder_i.value - self.prev_encoder_i.value)
                except IOError as error:
                    print(error)
                
                # update odometry uses values that require mutex
                # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)

                # Operations like += which involve a read and write are not atomic.
                #with self.x.get_lock():
    #                self.x.value+=1

                # to "lock" a whole set of operations, we can use a "mutex"
                #~ self.lock_odometry.acquire()
                #~ self.y.value+=1
                #~ self.th.value+=1
                #~ self.lock_odometry.release()

                v,w = self.readSpeed()
                deltaTh = w*self.P
                deltaS = v*self.P
                
                # Read the angle in mutual exclusion
                with self.th.get_lock():
                    angle = self.th.value + deltaTh/2
                    
                print("Delta th: ", deltaTh, ", Delta S: ", deltaS, ", w: ", w, ", v: ", v)
                print("Read angle in mutual exclusion: ", angle)
                    
                deltaX = deltaS*np.cos(angle)
                deltaY = deltaS*np.sin(angle)

                # Acquire lock, to avoid race conditions when calling `readOdometry`
                self.lock_odometry.acquire()
                self.x.value += deltaX
                self.y.value += deltaY
                self.th.value += deltaTh
                self.th.value = self.normaliza_rad(self.th.value)
                self.lock_odometry.release()
                
                print("Updated odometry. Current values: X= ", self.x.value, ", Y= ", self.y.value, ", TH= ", self.th.value)

                # save LOG
                # Need to decide when to store a log with the updated odometry ...
                # No need to read in mutual exclusion, since we JUST WROTE them and we
                # only perform writes on THIS thread
                csvwriter.writerow([tIni-tIniCSV, self.x.value, self.y.value, self.th.value*180.0/np.pi])
                
                ######## UPDATE UNTIL HERE with your code ########

                # High precission but used only to compute elapsed times (no use for absolute time)
                tEnd = time.perf_counter()
                
                time.sleep(self.P - (tEnd-tIni)) # Si este valor es negativo saldra error, pero queremos que pase para controlarlo

            # print("Stopping odometry ... X= ", x_val, ", Y= ", y_val, ", th= ", th_val)
            csvfile.close()
        
        except KeyboardInterrupt:
            # except the program gets interrupted by Ctrl+C on the keyboard.
            # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
            time.sleep(0.5)

    # Stop the odometry thread.
    def stopOdometry(self):
        print("THEEEE EEEEEND")
        self.finished.value = True
        self.BP.reset_all()
        

