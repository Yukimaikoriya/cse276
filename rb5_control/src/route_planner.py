#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Point

class RoutePlanner:
    def __init__(self, pub):
        self.pub = pub


def main():
    pub_motor = rospy.Publisher('/motor', Point, queue_size=1)
    rp = RoutePlanner(pub_motor)
    rospy.Subscriber('/rf_robot', Point, rp.callback, queue_size=1)
    rospy.spin()
