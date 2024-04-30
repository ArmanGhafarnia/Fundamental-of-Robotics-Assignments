import numpy as np
import math

x = 1.5
y = 2
teta = math.pi / 2
l = 0.5


def dc_kin(x, y, teta, v1, v2, t, l):
    R_NegTeta = [[np.cos(-teta), np.sin(-teta), 0],
                 [-np.sin(-teta), np.cos(-teta), 0],
                 [0, 0, 1]]

    xDotR = v1 / 2 + v2 / 2
    yDotR = 0
    tetaDotR = v1 / l - v2 / l
    cesiDotR = [[xDotR],
                [yDotR],
                [tetaDotR]]

    cesiDotI = np.dot(R_NegTeta, cesiDotR)
    xDot = cesiDotI[0][0]
    yDot = cesiDotI[1][0]
    tetaDot = cesiDotI[2][0]

    Xn = x + t * xDot
    Yn = y + t * yDot
    TETAn = teta + t * tetaDot
    result = [Xn, Yn, TETAn]

    return result


# Order 1
v1 = 0.3
v2 = 0.3
t = 3
resultOr1 = dc_kin(x, y, teta, v1, v2, t, l)
x = resultOr1[0]
y = resultOr1[1]
teta = resultOr1[2]
print("After Order 1:")
print(f"Xn = {x}, Yn = {y}, TETAn = {(teta*180) / math.pi}")


# Order 2
v1 = 0.1
v2 = -0.1
t = 1
resultOr2 = dc_kin(x, y, teta, v1, v2, t, l)
x = resultOr2[0]
y = resultOr2[1]
teta = resultOr2[2]
print("After Order 2:")
print(f"Xn = {x}, Yn = {y}, TETAn = {(teta*180) / math.pi}")

# Order 3
v1 = 0.2
v2 = 0
t = 2
resultOr3 = dc_kin(x, y, teta, v1, v2, t, l)
x = resultOr3[0]
y = resultOr3[1]
teta = resultOr3[2]
print("After Order 3:")
print(f"Xn = {x}, Yn = {y}, TETAn = {(teta*180) / math.pi}")