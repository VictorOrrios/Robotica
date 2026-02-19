# Author: 
# Date: 

import numpy as np
import time
import math
import cv2

import CoppeliaAPI.cfg as cfg
from CoppeliaAPI.cfg import csimLock

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

cfg = cfg.Cfg()


class _Motor:
    def __init__(self, client,  position = None):
        """ Creates the sensor with the handler """
        if position is not None: 
            self.handler = client.getObject('/CubeRob1/' + position + 'Motor' + str(cfg.ROBOT_ID))
            print('/CubeRob1/' + position + 'Motor' + str(cfg.ROBOT_ID))
            #dynamicallyEnabled = client.isDynamicallyEnabled(self.handler)
            client.setJointTargetVelocity(self.handler, 0.0)
            #print(dynamicallyEnabled);
        else: raise ValueError("Position of the motors is not specified")
        self.last_read = 0

    def read(self, client):

        readed = np.rad2deg(client.getJointPosition(self.handler))
        diff = readed - self.last_read
        self.last_read = readed
        return readed + diff
    #    return readed + diff * np.random.normal(0,0.025)
      
    def set_encoder(self, value, client):
        client.setJointPosition(self.handler, value)
        self.last_read = value

    def set_dps(self, dps, client):
        print("Set DPS:",dps)
        client.setJointTargetVelocity(self.handler, np.deg2rad(dps))
       
       

### SENSORES
class _Button:
    """ Simulates the TOUCH BrickPi sensor """
    def __init__(self, client, position = None):
        """ Creates the sensor with the handler """
        pass

    def read(self, client):
        """ Returns 0 if released, 1 if pressed """
        return 0

class _Ultrasonic:
    """ Simulates the  NXT_ULTRASONIC and EV3_ULTRASONIC_CM BrickPi sensors"""
    def __init__(self, client, position = "front"): # TODO: 2 ultrasonics
        """ Creates the sensor with the handler """
        #print('/CubeRob1/' + position + 'ProximitySensor' + str(cfg.ROBOT_ID))
        self.handler = client.getObject('/CubeRob1/' + position + 'ProximitySensor' + str(cfg.ROBOT_ID))

    def read(self, client):
        """ Uses the ultrasonic sensor to return the distance in cm """

        res, dist, point, obj, n = client.readProximitySensor( self.handler)

        #list = client.simxCheckProximitySensor(self.handler, "sim.handle_all", client.simxServiceCall())
        if(res == 0 ): return 255 # en plan return mucho porque no detectamos na
        else: return dist*100 + np.random.normal(0,dist) # por cada metro, una varianza de 1 cm

class _Gyro:
    """ Simulates the EV3_GYRO_ABS_DPS BrickPi sensors"""
    def __init__(self, client, position = "front"): 
        """ Creates the sensor with the handler """
        self.handler = client.getObject('/CubeRob1/' + 'GyroSensor')

    def read(self, client):
        """ Uses the gyro sensor to return the yaw in degrees"""
        orientation = client.getObjectOrientation(self.handler)  # [alpha, beta, gamma]
        gamma_deg = math.degrees(orientation[2])
        return [-gamma_deg, 0]    
    
class _Camera:
    """ Simulates a USB_CAMERA"""
    def __init__(self, client, position = "front"): 
        """ Creates the sensor with the handler """
        self.handler = client.getObject('/CubeRob1/' + 'visionCamera' + str(cfg.ROBOT_ID))
        
    def read(self, client):
        """ Uses the gyro sensor to return the yaw in degrees"""

        image, resolution = client.getVisionSensorImg(self.handler)
        width, height = resolution

        img_data = np.frombuffer(image, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
        img_data = img_data[::-1, :, :]
        img_data_cv2 = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)

        return img_data_cv2
    #img_data   


class _Light:
    """ Simulates the  NXT_LIGHT_ON BrickPi sensor"""
    def __init__(self, client, position = None):
        """ Creates the sensor with the handler """
        self.handler = client.getObject('/CubeRob1/' + 'visionLight' + str(cfg.ROBOT_ID))

    def read(self, client):
        """
        Uses the vision sensor to get the amount of light
        :return: the amount of light from 4000 (dark, no light) to 0 (bright, full light)
        """
        #ok, self.resolution, imageBytes = client.simxGetVisionSensorImage(self.handler, True, client.simxServiceCall())
        #img, [resX, resY] = sim.getVisionSensorImg(self.handler)
        #color = int(img[0])
        #return 4000 * (1 - color / 255)
        return 0

### FUNCIONES DE LA API
class BrickPi3:

    PORT_1 = 0x01
    PORT_2 = 0x02
    PORT_3 = 0x04
    PORT_4 = 0x08
    PORT_USB = 0x10
    PORT_A = 0x01
    PORT_B = 0x02
    PORT_C = 0x04
    PORT_D = 0x08


    class SENSOR_TYPE:
        TOUCH = _Button
        NXT_ULTRASONIC = _Ultrasonic
        EV3_ULTRASONIC_CM = _Ultrasonic
        EV3_GYRO_ABS_DPS = _Gyro
        USB_CAMERA = _Camera
        NXT_LIGHT_ON = _Light
        NXT_LIGHT_OFF = _Light # Its the same, but there isn't an auxiliar light
        #CUSTOM = _Custom

    class SENSOR_CUSTOM:
        PIN1_ADC = "pin1"

    ports_str = {
        "PORT_A": PORT_A,
        "PORT_B": PORT_B,
        "PORT_C": PORT_C,
        "PORT_D": PORT_D,
        "PORT_1": PORT_1,
        "PORT_2": PORT_2,
        "PORT_3": PORT_3,
        "PORT_4": PORT_4,
        "PORT_USB": PORT_USB,
    }
    
    def __init__(self):
        # RESET THE FRICKPI
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        ## PUERTOS DE SENSORES Y MOTORES
        self.ports_motor={
            #self.PORT_A : None, 
            #self.PORT_B : None,
            #self.PORT_C : None,
        }
        self.ports_sensor={
            #self.PORT_1 : None,
            #self.PORT_2 : None, 
            #self.PORT_3 : None, 
            #self.PORT_4 : None, 
            #self.PORT_USB : None, 
            
        }

        ## Associate motors to ports
        self.ports_motor[self.ports_str[cfg.MOTOR_CLAW]] = _Motor(self.sim, "claw")
        self.ports_motor[self.ports_str[cfg.MOTOR_LEFT]] = _Motor(self.sim, "left")
        self.ports_motor[self.ports_str[cfg.MOTOR_RIGHT]] = _Motor(self.sim, "right")

        # Launch simulation
        self.sim.startSimulation()
        time.sleep(0.5)

    def __getstate__(self):
        state = self.__dict__.copy()
        del state["client"]
        del state["sim"]
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

    def reset_all(self):
        # TODO: 
        """Reset the BrickPi. Set all the sensors' type to NONE, set the motors to float, and motors' limits and constants to default, and return control of the LED to the firmware."""
        with csimLock:
            self.sim.stopSimulation()
            del self.client
            del self.sim

    def reset_motor_encoder(self, ports):
        """
        Reset motor encoder(s) to 0

        Keyword arguments:
        ports -- The motor port(s). PORT_A, PORT_B, PORT_C, and/or PORT_D.
        """
        with csimLock:
            for port in [ports//i%2*i for i in [1,2,4,8] if ports//i%2 != 0]:
                self.ports_motor[port].set_encoder(0, self.sim)


    def offset_motor_encoder(self, ports, offset):
        """
        Offset a motor encoder

        Keyword arguments:
        ports -- The motor port(s). PORT_A, PORT_B, PORT_C, and/or PORT_D.
        offset -- The encoder offset

        You can zero the encoder by offsetting it by the current position
        """
        with csimLock:
            for port in [ports//i%2*i for i in [1,2,4,8] if ports//i%2 != 0]:
                self.ports_motor[port].set_encoder(self.ports_motor[port].read(self.sim) -  offset, self.sim)

    def get_motor_encoder(self, port):
        """
        Read a motor encoder in degrees

        Keyword arguments:
        port -- The motor port (one at a time). PORT_A, PORT_B, PORT_C, or PORT_D.

        Gets the encoder position in degrees, which contains the total rotated degrees since a reset 
        """
        with csimLock:
            return self.ports_motor[port].read(self.sim)

    def set_motor_dps(self, ports, dps):
        """
        Set the motor target speed in degrees per second

        Keyword arguments:
        ports -- The motor port(s). PORT_A, PORT_B, PORT_C, and/or PORT_D.
        dps -- The target speed in degrees per second
        """
        with csimLock:
            for port in [ports//i%2*i for i in [1,2,4,8]  if ports//i%2 != 0]:
                self.ports_motor[port].set_dps(dps, self.sim)

    def set_sensor_type(self, ports, tipo_sensor, params = 0, position=None):
        """
        Set the sensor type on a port

        Keyword arguments:
        ports -- The sensor port(s). PORT_1, PORT_2, PORT_3, and/or PORT_4.
        type -- The sensor type
        params = 0 -- the parameters needed for some sensor types.

        params is used for the following sensor types:
            CUSTOM -- a 16-bit integer used to configure the hardware.
            I2C -- a list of settings:
                params[0] -- Settings/flags
                params[1] -- target Speed in microseconds (0-255). Realistically the speed will vary.
                if SENSOR_I2C_SETTINGS_SAME flag set in I2C Settings:
                    params[2] -- Delay in microseconds between transactions.
                    params[3] -- Address
                    params[4] -- List of bytes to write
                    params[5] -- Number of bytes to read
        """        
        with csimLock:
            for port in [ports//i%2*i for i in [1,2,4,8] if ports//i%2 != 0]:
                if(cfg.ALT_ULTRASOUND_PORT != "None" and self.ports_str[cfg.ALT_ULTRASOUND_PORT] == port):
                    self.ports_sensor[port] = tipo_sensor(self.sim, cfg.ALT_ULTRASOUND_DIR)
                else:
                    self.ports_sensor[port] = tipo_sensor(self.sim)

 
    def get_sensor(self, port):
        """
        Read a sensor value

        Keyword arguments:
        port -- The sensor ports (one at a time). PORT_1, PORT_2, PORT_3, or PORT_4.

        Returns the value(s) for the specified sensor.
            The following sensor types each return a single value:
                NONE ----------------------- 0
                TOUCH ---------------------- 0 or 1 (released or pressed)
                NXT_TOUCH ------------------ 0 or 1 (released or pressed)
                EV3_TOUCH ------------------ 0 or 1 (released or pressed)
                NXT_ULTRASONIC ------------- distance in CM
                NXT_LIGHT_ON  -------------- reflected light
                NXT_LIGHT_OFF -------------- ambient light
                NXT_COLOR_RED -------------- red reflected light
                NXT_COLOR_GREEN ------------ green reflected light
                NXT_COLOR_BLUE ------------- blue reflected light
                NXT_COLOR_OFF -------------- ambient light
                EV3_GYRO_ABS --------------- absolute rotation position in degrees
                EV3_GYRO_DPS --------------- rotation rate in degrees per second
                EV3_COLOR_REFLECTED -------- red reflected light
                EV3_COLOR_AMBIENT ---------- ambient light
                EV3_COLOR_COLOR ------------ detected color
                EV3_ULTRASONIC_CM ---------- distance in CM
                EV3_ULTRASONIC_INCHES ------ distance in inches
                EV3_ULTRASONIC_LISTEN ------ 0 or 1 (no other ultrasonic sensors or another ultrasonic sensor detected)
                EV3_INFRARED_PROXIMITY ----- distance 0-100%
                USB_CAMERA ----------------- image

            The following sensor types each return a list of values
                CUSTOM --------------------- Pin 1 ADC (5v scale from 0 to 4095), Pin 6 ADC (3.3v scale from 0 to 4095), Pin 5 digital, Pin 6 digital
                I2C ------------------------ the I2C bytes read
                NXT_COLOR_FULL ------------- detected color, red light reflected, green light reflected, blue light reflected, ambient light
                EV3_GYRO_ABS_DPS ----------- absolute rotation position in degrees, rotation rate in degrees per second
                EV3_COLOR_RAW_REFLECTED ---- red reflected light, unknown value (maybe a raw ambient value?)
                EV3_COLOR_COLOR_COMPONENTS - red reflected light, green reflected light, blue reflected light, unknown value (maybe a raw value?)
                EV3_INFRARED_SEEK ---------- a list for each of the four channels. For each channel heading (-25 to 25), distance (-128 or 0 to 100)
                EV3_INFRARED_REMOTE -------- a list for each of the four channels. For each channel red up, red down, blue up, blue down, boadcast

        """
        with csimLock:
            return self.ports_sensor[port].read(self.sim)


