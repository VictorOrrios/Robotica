#!/usr/bin/python
# -*- coding: UTF-8 -*-
import brickpi3
import time
import numpy as np

BP = brickpi3.BrickPi3()

# Configure the gyro sensor on PORT 4
# We use SENSOR_TYPE.EV3_GYRO_ABS_DPS to get both angle and degrees per second
GYRO_PORT = BP.PORT_3
BP.set_sensor_type(GYRO_PORT, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)

print("Configuring Gyro... Please keep the robot still.")
time.sleep(2.0) # Wait for calibration

try:
    print("Starting gyro test. Press Ctrl+C to stop.")
    while True:
        try:
            # Each call to get_sensor returns a list of values
            # For EV3_GYRO_ABS_DPS, it returns [angle, dps]
            value = BP.get_sensor(GYRO_PORT)
            print("Gyro Angle: {:d} deg, Speed: {:d} dps, value: {}".format(value[0], value[1], value))
        except brickpi3.SensorError as error:
            print("Sensor Error: ", error)
        
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Stopping gyro test.")
    BP.reset_all()
