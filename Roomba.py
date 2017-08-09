# Oneal Abdulrahim
# Amin Zeiaee
# iRobot Create: ROOMBA Control System

import create
import math

''' ----- Preliminary Setup & Constants ----- '''

# known starting constants for iRobot
beta = 100 # heading of the robot (will change)
D_WHEEL = 23.5 # distance between wheels [cm]
R_WHEEL =  3.6 # radius of wheels [cm]

# initialize robot object from create library
Roomba = create.Create("COM4") #port = COM4 (RPi & Windows)


''' ----- Control & Calculation Methods ----- '''

def Control(xi, yi, xf, yf):
    """ Returns the necessary velocity and angular
        velocity based on position values.

        @param    xi    initial x value
                  yi    initial y value
                  xf    final x value
                  yf    final y value
        @return   v     velocity
                  w     angular velocity """

    ''' Lyapunov based control '''
    # conversion to polar
    rho = math.pow(math.pow(xf-xi,2)+math.pow(yf-yi,2),1/2)

    # four quadrant inverse tangent
    # beta = 100 (current heading of robot)
    gamma = math.atan2(yi,xi)-beta+math.pi

    v = rho*math.cos(gamma)
    w = 3*gamma+0.5*math.sin(2*gamma)*(3+2/gamma)

    return v,w

def Position(v, w, xc, yc, beta, l_sensor, r_sensor):
    """ Updates robot position, calculates new values as needed.
        Returns new values for position & updates sensor values.

        @param    v    velocity
                  w    angular velocity
                  xc   current x value
                  yc   current y value
        @return   xn   new x value
                  yn   new y value
                  bnew updated beta value """

    # from current values
    Roomba.go(v,w)
    
    # LEFT/RIGHT encoder perpetually increase, 508.8 transistions per wheel rev
    l_sensor_d = Roomba.getSensor("LEFT_ENCODER") - l_sensor
    r_sensor_d = Roomba.getSensor("RIGHT_ENCODER") - r_sensor

    # conversion from encoder value to centimeters
    r_wheel = l_sensor_d * (72.0 * math.pi / 508.8) / 10
    l_wheel = r_sensor_d * (72.0 * math.pi / 508.8) / 10

    # updating values and distance
    bn = ((r_wheel - l_wheel) / D_WHEEL) + beta
    xn = ((1/2 * (r_wheel + l_wheel)) * math.cos(bn)) + xc
    yn = ((1/2 * (r_wheel + l_wheel)) * math.sin(bn)) + yc
    
    return xn, yn, bn, Roomba.getSensor("LEFT_ENCODER"), Roomba.getSensor("RIGHT_ENCODER")

''' ----- Moving iRobot from one point to another ----- '''
x_start = -2
y_start = -1
x_final = 0
y_final = 0
"""
x_start = int(input("START X: "))
y_start = int(input("START Y: "))

x_final = int(input("FINAL X: "))
y_final = int(input("FINAL Y: "))
"""
left_sensor = Roomba.getSensor("LEFT_ENCODER")
right_sensor = Roomba.getSensor("RIGHT_ENCODER")

while x_start != x_final and y_start != y_final:
    v, w = Control(x_start, y_start, x_final, y_final)
    x_start, y_start, beta, left_sensor, right_sensor = Position(v, w, x_start, y_start, beta, left_sensor, right_sensor)
    Roomba.stop()
Roomba.shutdown()
