import serial
import math
import create
import ControllerRobust
import DeadReckon
# import

# global r is robot object from create library
r = create.Create("COM4")

# Final and initial values of (x, y) given as cartesian coordinates
X_FINAL = 0
Y_FINAL = 0
X = -2
Y = -1
BETA = 0
Um = 100
LC = r.getSensor("LEFT_ENCODER")
RC = r.getSensor("RIGHT_ENCODER")

# tracking data & pathing in text file
xf = open('xf', 'w')
yf = open('yf', 'w')
# Vf = open('Vf','w')
# Wf = open('Wf','w')

while X != X_FINAL and Y != Y_FINAL:
    (V, W) = ControllerRobust.Control(X, Y, Um, X_FINAL, Y_FINAL)
    (X, Y, BETA, LC, RC) = DeadReckon.Position(V, W, X, Y, BETA, LC, RC, r)
    xf.write(str(X))
    xf.write('\n')
    yf.write(str(Y))
    yf.write('\n')
    # Vf.write(str(V))
    # Wf.write(str(W))
    print(Y)
input("Press any key to exit...")
r.shutdown()

xf.close()
yf.close()
# Vf.close()
# Wf.close()
