from .robotarm import RobotArm
from math import pi
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

class Room(object):
    def __init__(self, parent_fig, shape=((-1, 1),(-1,1),(0,2)), trace=False, tracejoint=-1):
        self.parent_fig = parent_fig
        self.ax = self.parent_fig.add_subplot(111, projection='3d')
        self.active_robots = {}
        self.robot_controls = {}
        self.shape = shape

        self.trace = trace
        self.tracePathX = []
        self.tracePathY = []
        self.tracePathZ = []
        self.tracejoint = tracejoint

    def add_robot(self, coords, rotation=(0,0), name='DefaultBot'):
        '''Adds a robot at the provided coordinates with the provided name'''
        if not name in self.active_robots:
            self.active_robots[name] = RobotArm(coords, rotation=rotation)
            self.robot_controls[name] = [None]
            return self.active_robots[name]
        else:
            print("Robot with that name already in room")
            return None

    def add_controls(self, name, controls):
        '''
        Adds the defined control scheme,to the robot
        Control schemes are functions of the form:
            f(time, robot) -> None
        '''
        if name in self.active_robots:
            self.robot_controls[name] = controls
        else:
            print("No robot exists for provided name")

    def draw_room(self):
        if plt.fignum_exists(self.parent_fig.number):
            blankPlot = self.ax.plot([],[], [], animated=True)
            self.ax.set_xlim(self.shape[0][0],self.shape[0][1])
            self.ax.set_ylim(self.shape[1][0],self.shape[1][1])
            self.ax.set_zlim(self.shape[2][0],self.shape[2][1])
            return blankPlot
        else:
            raise Exception("Parent figure for room is closed")

    def update(self, time):
        if time == 0:
            self.tracePathX = []
            self.tracePathY = []
            self.tracePathZ = []
        self.ax.clear()
        self.ax.set_xlim(self.shape[0][0],self.shape[0][1])
        self.ax.set_ylim(self.shape[1][0],self.shape[1][1])
        self.ax.set_zlim(self.shape[2][0],self.shape[2][1])

        ## Adding more concrete profiles needs to go here and in the definition of what a control is
        for name, robot in self.active_robots.items():
            # Apply the control scheme to the robot
            self.robot_controls[name](time, robot)
            xdata = []
            ydata = []
            zdata = []
            jc = robot.jointLocations
            # Label the joint coordinates for the end effector
            [
            self.ax.text(
                x[0],x[1],x[2], " x:{:05.2f}\n y:{:05.2f}\n z:{:05.2f}".format(
                    x[0], x[1], x[2]), fontsize=8) 
            for x in jc[-2:]
            ]
            # Add the robots name to the plt
            self.ax.text(jc[0][0],jc[0][0],jc[0][0]-.5, str(name), fontsize=8)
            # Add each joints current coordinates to the data arrays
            [(xdata.append(x[0]),ydata.append(x[1]),zdata.append(x[2])) for x in jc] 

            # If tracing joints then log the joints positions. need to redo trace
            if self.trace:
                self.tracePathX.append(xdata[self.tracejoint])
                self.tracePathY.append(ydata[self.tracejoint])
                self.tracePathZ.append(zdata[self.tracejoint])
                self.ax.plot(self.tracePathX,self.tracePathY,self.tracePathZ, color='red',linestyle='--')

        return self.ax.plot(xdata,ydata,zdata, color='green', marker='o')

    def animate(self):
        self.ani = FuncAnimation(self.parent_fig, self.update, 
            interval=50,repeat_delay=500, frames=np.linspace(0,20,201))

        # self.ani.save("test.gif")

        plt.show()