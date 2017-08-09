import serial
import math
import create
import time

def Position(V, W, X, Y, Beta, LC, RC, r):
    """ The Position method updates the robot's current position.
        Calculation is based on current location and current sensor
        values received from robot.

        This method is called continuously as the robot paths to
        the new location.

        Returns new values for current position and sensor values.
    """
    d = 23.5 # distance between wheels (cm)
    rad = 3.6 # radius of wheels (cm)

    # iCreate Robot library uses left and right wheel VELOCITY values
    LeftWheelVel = ((V - ((W * d) / 2)) / rad) * 3.6 / (2 * math.pi) * 100
    RightWheelVel = (V * 2 + W * d / (2 * rad)) * 3.6 / (2 * math.pi) * 100

    # r is robot object from create library
    r.driveDirect(LeftWheelVel, RightWheelVel) # assign calculated velocities
    
    LCnew = r.getSensor("LEFT_ENCODER") # recieve updated sensor 
    RCnew = r.getSensor("RIGHT_ENCODER")
    DLC = LCnew - LC # update sensor values from previously record
    DRC = RCnew - RC
    # L is Left?
    DLWheel = DLC * (72.0 * math.pi / 508.8) / 10
    DRWheel = DRC * (72.0 * math.pi / 508.8) / 10
    BetaNew = ((DRWheel - DLWheel) / d) + Beta
    XNew = ((1/2 * (DRWheel + DLWheel)) * math.cos(BetaNew)) + X
    YNew = ((1/2 * (DRWheel + DLWheel)) * math.sin(BetaNew)) + Y
    return(XNew, YNew, BetaNew, LCnew, RCnew)
