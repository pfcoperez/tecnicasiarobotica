# -*- coding: utf-8 -*-b 

"""
A PyrobotSimulator world. A large room with two robots and
two lights.

(c) 2005, PyroRobotics.org. Licensed under the GNU GPL.
"""

from pyrobot.simulators.pysim import *
from math import pi

def INIT():
    # (width, height), (offset x, offset y), scale:
    sim = TkSimulator((221,233), (11,210), 40.357554)
    # x1, y1, x2, y2 in meters:
    sim.addBox(0, 0, 3.5, 3.5)
    sim.addBox(1.0,1.0,2.5,2.5);
    #sim.addBox(0.36,2.8,0.9,3.12);
    # port, name, x, y, th, bounding Xs, bounding Ys, color
    # (optional TK color name):
    # En la construcción TkPioneer, se da (nombre, x, y, angulo en rads)
    sim.addRobot(60000, TkPioneer("RedPioneer",
                                  0.3, 0.3,-pi/4.0 ,
                                  ((.225, .225, -.225, -.225),
                                   (.175, -.175, -.175, .175)),
                                  "red"))
    # add some sensors:
    sim.robots[0].addDevice(PioneerFrontSonars())
    return sim
