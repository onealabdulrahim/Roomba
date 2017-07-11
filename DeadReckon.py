import serial
import math
import create


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
    LCnew = r.getSensor("LEFT_ENCODER") # recieve updated sensor values
    RCnew = r.getSensor("RIGHT_ENCODER")
    DLC = LCnew - LC # update sensor values from previously record
    DRC = RCnew - RC
    DLWheel = DLC * 2 * math.pi / 508.8
    DRWheel = DRC * 2 * math.pi / 508.8
    BetaNew = ((DRWheel - DLWheel) * (rad / d)) / 2 + Beta
    XNew = (((rad / 2) * (DRWheel + DLWheel)) * math.cos(BetaNew)) + X
    YNew = (((rad / 2) * (DRWheel + DLWheel)) * math.sin(BetaNew)) + Y
    return(XNew, YNew, BetaNew, LCnew, RCnew)
