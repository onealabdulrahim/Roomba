import serial
import math
import create


def Position(V, W, X, Y, Beta, LC, RC, r):
    d = 0.235
    rad = 0.036

    LeftWheelVel = ((V - ((W * d) / 2)) / rad) * 0.036 / (2 * math.pi) * 100
    RightWheelVel = (V * 2 + W * d / (2 * rad)) * 0.036 / (2 * math.pi) * 100
    r.driveDirect(LeftWheelVel, RightWheelVel)
    LCnew = r.getSensor("LEFT_ENCODER")
    RCnew = r.getSensor("RIGHT_ENCODER")
    DLC = LCnew - LC
    DRC = RCnew - RC
    DLWheel = DLC * 2 * math.pi / 508.8
    DRWheel = DRC * 2 * math.pi / 508.8
    BetaNew = ((DRWheel - DLWheel) * (rad / d)) / 2 + Beta
    XNew = (((rad / 2) * (DRWheel + DLWheel)) * math.cos(BetaNew)) + X
    YNew = (((rad / 2) * (DRWheel + DLWheel)) * math.sin(BetaNew)) + Y
    return(XNew, YNew, BetaNew, LCnew, RCnew)
