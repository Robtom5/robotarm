from robotarm import Room, make_dh
import math
from math import pi, atan2, sqrt, sin, acos
import numpy as np
from numpy import arccos
import matplotlib.pyplot as plt
from scipy import interpolate as interp


points = [
    [[0.5, 0, 0.0], 0],
    [[-0.4, .4, 0.5], 5],
    [[-0.4, 0.1, 0.2], 10],
    [[0,-0.3,0.6], 15],
    [[0.5, 0, 0.0], 20]
    ]
points.sort(key=lambda x: x[1])

def points_converter(points):
    xs = [points[i][0][0] for i in range(len(points))]
    ys = [points[i][0][1] for i in range(len(points))]
    zs = [points[i][0][2] for i in range(len(points))]
    ts = [points[i][1] for i in range(len(points))]
    return ([xs,ys,zs], ts)

coord, timestamps = points_converter(points)

tck, u = interp.splprep(coord, s=0, u=timestamps)

def points_to_route(points, time):
    lastPoint = next(point for point in reversed(points) if point[1] <= time)
    nextPoint = next(point for point in points if point[1] >= time)

    interval = nextPoint[1]-lastPoint[1]
    offsetTime = time-lastPoint[1]

    lastCoord = np.array(lastPoint[0])
    nextCoord = np.array(nextPoint[0])
    if interval == 0:
        return lastCoord
    return (lastCoord + (offsetTime/interval * (nextCoord - lastCoord)))


def add_cw_bot(room, control):
    cw = room.add_robot((0,0,0),(0,0),name = 'ExampleBot')
    cw.add_revolute(alpha=pi/2)
    cw.add_revolute(a=.5)
    cw.add_revolute(a=.5, alpha = -pi/2)
    room.add_controls('ExampleBot', control)
    return cw

def simple_control(time, robot):
    timeLoc = interp.splev(time, tck)
    robot.set_arm_values(simple_inverse(timeLoc, robot)[0])


def simple_inverse(point, robot):
    end_eff = point
    link_len = 0.5

    theta1_front = atan2(end_eff[1], end_eff[0])
    theta1_back = atan2(end_eff[1], end_eff[0])+pi

    Cxy = sqrt(end_eff[0]**2 + end_eff[1]**2)
    r = sqrt(Cxy**2 + (end_eff[2]**2))

    theta3 = pi - acos(((link_len**2) + (link_len**2) - (r**2))/(2*link_len*link_len))

    theta2a = acos(((link_len**2) - (link_len**2) +( r**2))/(2*r*link_len))
    theta2b = atan2(end_eff[2], Cxy)

    elbupF = [theta1_front, theta2a+theta2b, -theta3]
    elbdownF = [theta1_front, theta2a+theta2b, theta3]

    return (elbupF, elbdownF)

    # return simple_elbow_ang(t1cf, theta1_front, t01f, t0c)

def main():
    figure = plt.figure()
    theRoom = Room(figure, trace=True)
    cw_bot = add_cw_bot(theRoom, simple_control)
    # cw_bot.set_arm_values([pi, None, None, None, None, None])
    # calc_inv_kinematics([0.5,0.5,0.5], cw_bot)
    theRoom.animate()


main()