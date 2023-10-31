#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Point
import numpy as np

from move_planner import MovePlanner

class RoutePlanner:
    POOLSIZE = 5
    FWD = 0
    TURN = 1
    STOP = -1
    PRECISION = 0.01
    ALIGN_FACTOR = 5
    ANGLE_PRECISION = 0.01
    def __init__(self, pub, verbose=True):
        self.pub = pub
        self.direction = 0
        self.x = 0
        self.y = 0
        self.verbose = verbose
        self.pool = [(0,0,0) for _ in range(RoutePlanner.POOLSIZE)]
        self.last_motor_command = self.STOP
    # use last 5 points for frame estimation
    def update(self, pos):
        self.pool.append((pos.x, pos.y, pos.z))
        self.pool = self.pool[1:]
        self.x, self.y, self.direction = np.average(self.pool, axis=0)
        if self.verbose:
            rospy.loginfo('EX=%s\tEY=%s\tER=%s' % (self.x, self.y, self.direction))
    def callback(self, pos):
        if self.verbose:
            rospy.loginfo('X=%s\tY=%s\tR=%s' % (pos.x, pos.y, pos.z))
        self.update(pos)
    def post_process(self, wpt):
        # align to wpt direction
        r = rospy.Rate(10)
        while True:
            align, value = self.check_alignment(wpt[2])
            if align:
                self.send(self.TURN, value)
            else:
                break
            r.sleep()
        # wait 1s on waypoint
        self.send(self.STOP)
        rospy.sleep(1)
    def on_waypoint(self, wpt):
        return (abs(self.x - wpt[0]) <= self.PRECISION and 
            abs(self.y - wpt[1]) <= self.PRECISION)
    def check_alignment(self, target):
        if isinstance(target, list):
            x, y, _ = target
            x -= self.x
            y -= self.y
            c, s = np.cos(self.direction), np.sin(self.direction)
            x, y = x * c - y * s, x * s + y * c
            value = np.arctan2(y, x)
            align = abs(y) > self.ALIGN_FACTOR * self.PRECISION
            return (align, value)
        else:
            value = target - self.direction
            return (abs(value) > self.ANGLE_PRECISION, value)
    def send(self, cmd, value=0):
        if cmd == self.FWD:
            if self.verbose:
                rospy.loginfo('FWD')
            self.pub.publish(MovePlanner.CMD_FWD)
            return
        elif cmd == self.TURN:
            if value > 0:
                if self.verbose:
                    rospy.loginfo('RT')
                self.pub.publish(MovePlanner.CMD_RT)
                return
            elif value < 0:
                if self.verbose:
                    rospy.loginfo('LT')
                self.pub.publish(MovePlanner.CMD_LT)
                return
        if self.verbose:
            rospy.loginfo('STOP')
        self.pub.publish(MovePlanner.CMD_STOP)
        
    def main(self, waypoints):
        self.waypoints = waypoints
        self.waypoints.reverse()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            wpt = self.waypoints[-1]
            if self.on_waypoint(wpt):
                self.post_process(wpt)
                self.waypoints.pop()
                if not self.waypoints:
                    return
            wpt = self.waypoints[-1]
            # check alignment
            align, value = self.check_alignment(wpt)
            if align:
                self.send(self.TURN, value)
            else:
                # alignment ok. forward!
                self.send(self.FWD)
            r.sleep()


def main():
    rospy.init_node('route_planner')
    pub_command = rospy.Publisher('/motor', Point, queue_size=1)
    rp = RoutePlanner(pub_command)
    rospy.Subscriber('/rf_robot', Point, rp.callback, queue_size=1)
    waypoint = [[0.0,0.0,0.0], [0.5,0.0,0.0],[0.5,1.0,np.pi],[0.0,0.0,0.0]]
    rp.main(waypoint)

if __name__ == '__main__':
    main()
