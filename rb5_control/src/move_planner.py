#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Point

def point(x, y, z):
    ret = Point()
    ret.x = x
    ret.y = y
    ret.z = z
    return ret

class MovePlanner:
    STOP = 0
    FWD = 1
    LT = 2
    RT = 3
    CMD_STOP = point(0, 0, 0)
    CMD_FWD = point(1, 0, 0)
    CMD_LT = point(0, 1, 0)
    CMD_RT = point(0, 0, 1)
    def __init__(self, pub):
        pass




def main():
    pass


if __name__ == '__main__':
    main()
