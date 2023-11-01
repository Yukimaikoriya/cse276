#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Point
import numpy as np

from move_planner import MovePlanner

def dist(x1, y1, x2, y2):
    return np.linalg.norm(np.array([x1-x2, y1-y2]))

class RoutePlanner:
    POOLSIZE = 10
    POS_AVG = 3
    DIRAVG = 5
    FWD = 0
    TURN = 1
    STOP = -1
    PRECISION = 0.1
    ALIGN_FACTOR = 1
    ANGLE_PRECISION = 0.2
    P_05V = 0.06
    P_A2T = 5.4 / np.pi
    def __init__(self, pub, verbose=True):
        self.pub = pub
        self.direction = 0
        self.x = 0
        self.y = 0
        self.verbose = verbose
        self.pool = [(0,0,0) for _ in range(RoutePlanner.POOLSIZE)]
        self.dirs = [0 for _ in range(self.DIRAVG)]
        self.last_motor_command = self.STOP
        self.alignment_in_process = False
    # use last POOLSIZE points for frame estimation
    def update(self, pos):
        self.pool.append((pos.x, pos.y, pos.z))
        self.pool = self.pool[1:]
        self.x, self.y, _ = np.average(self.pool[-self.POS_AVG:], axis=0)
        self.update_direction()
        if self.verbose:
            rospy.loginfo('EX=%.3f\tEY=%.3f\tER=%.3f' % (self.x, self.y, self.direction))
    # direction update is complicated
    def update_direction(self):
        if self.alignment_in_process:
            return
        # first, we update only if we traveled at least 0.5v in the last window
        d = dist(self.pool[0][0], self.pool[0][1], self.pool[-1][0], self.pool[-1][1])
        if d < self.P_05V:
            if self.verbose:
                rospy.loginfo('DIRECTION NOT RECORDED D=%.3f' % d)
            rec_dir = self.direction
        else:
            rec_dir = np.arctan2(self.pool[-1][1]-self.pool[0][1], self.pool[-1][0]-self.pool[0][0])
            if self.verbose:
                rospy.loginfo('DIRECTION RECORDED %.3f' % rec_dir)
        self.dirs.append(rec_dir)
        self.dirs = self.dirs[1:]
        # if 5 consecutive recorded direction differ from self.direction
        # by more than ANGLE_PRECISION, update self.direction to the average of the 5
        if np.min(np.abs(np.array(self.dirs) - self.direction)) > self.ANGLE_PRECISION:
            self.direction = np.average(self.dirs)
            if self.verbose:
                rospy.loginfo('DIRECTION UPDATED: %.3f' % self.direction)
    def callback(self, pos):
        if self.verbose:
            rospy.loginfo('X=%.3f\tY=%.3f\tR=%.3f' % (pos.x, pos.y, pos.z))
        self.update(pos)
    def post_process(self, wpt):
        # align to wpt direction
        align, value =  self.check_alignment(wpt[2])
        if align:
            self.align(value)
        # wait 1s on waypoint
        rospy.sleep(1)
    def on_waypoint(self, wpt):
        ret = (abs(self.x - wpt[0]) <= self.PRECISION and 
            abs(self.y - wpt[1]) <= self.PRECISION)
        if self.verbose:
            rospy.loginfo('On %s: %s' % (wpt, ret))
        return ret
    def check_alignment(self, target):
        if isinstance(target, list):
            x, y, _ = target
            x -= self.x
            y -= self.y
            c, s = np.cos(-self.direction), np.sin(-self.direction)
            x, y = x * c - y * s, x * s + y * c
            if self.verbose:
                rospy.loginfo('AlmtChk: X=%.3f Y=%.3f' % (x, y))
            value = np.arctan2(y, x)
            align = abs(y) > self.ALIGN_FACTOR * self.PRECISION
            #if not align and x < -self.PRECISION:
             #   return (True, np.pi)
            return (align, value)
        else:
            value = target - self.direction
            return (abs(value) > self.ANGLE_PRECISION, value)
    def align(self, value):
        self.send(self.STOP)
        dt = self.direction + value
        if self.verbose:
            rospy.loginfo('Align %s' % value)
        delay = abs(value) * self.P_A2T
        self.alignment_in_process = True
        self.send(self.TURN, value)
        rospy.sleep(delay)
        self.send(self.STOP)
        self.alignment_in_process = False
        self.direction = dt
        self.dirs = [dt for _ in range(self.DIRAVG)]
    def send(self, cmd, value=0):
        if cmd == self.FWD:
            if self.verbose:
                rospy.loginfo('FWD')
            self.pub.publish(MovePlanner.CMD_FWD)
            return
        elif cmd == self.TURN:
            if value > 0:
                if self.verbose:
                    rospy.loginfo('LT')
                self.pub.publish(MovePlanner.CMD_LT)
                return
            elif value < 0:
                if self.verbose:
                    rospy.loginfo('RT')
                self.pub.publish(MovePlanner.CMD_RT)
                return
        if self.verbose:
            rospy.loginfo('STOP')
        self.pub.publish(MovePlanner.CMD_STOP)

    def main1(self, _):
        # calibration
        rospy.sleep(1)
        self.align(np.pi/2)
        
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
                self.align(value)
            else:
                # alignment ok. forward!
                self.send(self.FWD)
            r.sleep()


def main():
    rospy.init_node('route_planner')
    #rospy.sleep(10) # prepare video
    pub_command = rospy.Publisher('/motor', Point, queue_size=1)
    rp = RoutePlanner(pub_command)
    rospy.Subscriber('/rf_robot', Point, rp.callback, queue_size=1)
    waypoint = [[0.0,0.0,0.0], [0.5,0.0,0.0],[0.5,1.0,np.pi],[0.0,0.0,0.0]]
    rp.main(waypoint)

if __name__ == '__main__':
    main()
