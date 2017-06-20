import serial
import math
import create
import ControllerRobust
import DeadReckon
# import 

# global r
r = create.Create("COM4")

XFinal = 0
YFinal = 0
X = -2;
Y = -1;
Beta = 0;
Um = 100;
LC = r.getSensor("LEFT_ENCODER")
RC = r.getSensor("RIGHT_ENCODER")

xf = open('xf','w')
yf = open('yf','w')
# Vf = open('Vf','w')
# Wf = open('Wf','w')

while (X != XFinal and Y != YFinal):
	(V,W) = ControllerRobust.Control(X,Y,Um,XFinal,YFinal)
	(X,Y,Beta,LC,RC) = DeadReckon.Position(V,W,X,Y,Beta, LC, RC,r)
	xf.write(str(X))
	xf.write('\n')
	yf.write(str(Y))
	yf.write('\n')
	# Vf.write(str(V))
	# Wf.write(str(W))
	print(Y)
input("PressEnterToExit")
r.shutdown()
	
xf.close()
yf.close()
# Vf.close()
# Wf.close()
