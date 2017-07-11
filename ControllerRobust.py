import math

def Control(X, Y, Um, XFinal, YFinal):
    """ This Control method takes the current position and final desired positions
        as parameters and calculates the path the robot will take. The control
        values are then adjusted to fit within the allowed range.
    """

    # values from predetermined calculations
    norm = math.pow(math.pow((X - XFinal), 2) + math.pow((Y - YFinal), 2), (1 / 2))
    Vd = 0.1 / (1 + 1 / (3000 * math.pow(norm, 2)))
    a = 0.1
    fprime = -1.5 * math.pow(X, 2) - 2.5 * X
    eps = 0.5

    # X and Y directions
    X_d = (a * (3 * X + 5 / 2) * ((3 * math.pow(X, 2)) / 2 + (5 * X) / 2)) / (math.pow((math.pow(((3 * math.pow(X, 2)) / 2 + (5 * X) / 2), 2) + 1), (3 / 2))) + 1
    Y_d = (a * (3 * X + 5 / 2)) / (math.pow((math.pow(((3 * math.pow(X, 2)) / 2 + (5 * X) / 2), 2) + 1), (1 / 2))) - (3 * math.pow(X, 2)) / 2 - (5 * X) / 2 - ((a * (3 * X + 5 / 2) * (math.pow(((3 * math.pow(X, 2)) / 2 + (5 * X) / 2), 2)))) / \
        (math.pow((math.pow(((3 * math.pow(X, 2)) / 2 + (5 * X) / 2), 2) + 1), (3 / 2)))

    # respective angles
    theta_d = math.atan2(Y_d, X_d) + eps * math.copysign(1,
                                                         (Y - (-0.5 * math.pow(X, 3) - 1.25 * math.pow(X, 2))))
    ustar1 = Vd * math.cos(math.atan2(fprime, 1) - theta_d)
    ustar2 = Vd * math.sin(math.atan2(fprime, 1) - theta_d) / a

    # controlling range
    if abs(ustar1) <= Um and abs(ustar2) <= Um:
        control1 = ustar1
        control2 = ustar2
    elif abs(ustar1) <= Um and ustar(2) > Um:
        control1 = (Um / ustar2) * ustar1
        control2 = Um
    elif abs(ustar1) <= Um and ustar2 < -Um:
        control1 = abs(Um / ustar2) * ustar1
        control2 = -Um
    elif ustar1 > Um and abs(ustar2) <= Um:
        contro11 = Um
        control2 = abs(Um / ustar1) * ustar2
    elif ustar1 < - Um and abs(ustar2) <= Um:
        control1 = -Um
        control2 = abs(Um / ustar1) * ustar2
    elif ustar1 > Um and ustar > Um and abs(ustar1) > abs(ustar2):
        control1 = Um
        control2 = abs(Um / ustar1) * ustar2
    elif ustar1 > Um and ustar2 > Um and abs(ustar1) < abs(ustar2):
        control1 = (Um / ustar2) * ustar1
        control2 = Um
    elif ustar1 > Um and ustar2 < -Um and abs(ustar1) > abs(ustar2):
        control1 = Um
        control2 = abs(Um / ustar1) * ustar2
    elif ustar1 > Um and ustar2 < -Um and abs(ustar1) < abs(ustar2):
        control1 = abs(Um / ustar2) * ustar1
        control2 = -Um
    elif ustar1 < -Um and ustar2 > Um and abs(ustar1) > abs(ustar2):
        control1 = -Um
        control2 = abs(Um / ustar1) * ustar2
    elif ustar1 < -Um and ustar2 > Um and abs(ustar1) < abs(ustar2):
        control1 = (Um / ustar2) * ustar1
        control2 = Um
    elif ustar1 < -Um and ustar2 < -Um and abs(ustar1) > abs(ustar2):
        control1 = -Um
        control2 = abs(Um / ustar1) * ustar2
    elif ustar1 < -Um and ustar2 < -Um and abs(ustar1) < abs(ustar2):
        control1 = abs(Um / ustar2) * ustar1
        control2 = -Um
    V = control1
    W = control2
    return (V, W)
