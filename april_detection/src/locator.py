#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from april_detection.msg import AprilTagDetectionArray
import tf.transformations as tft
import numpy as np

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

class LocatorNode:
    @staticmethod
    def pose_diff(pos_start, pos_end):
        eulerof = lambda o : np.array(tft.euler_from_quaternion([o.x, o.y, o.z, o.w]))
        ret = Twist()
        ret.linear.x = pos_end.position.x - pos_start.position.x
        ret.linear.y = pos_end.position.y - pos_start.position.y
        ret.linear.z = pos_end.position.z - pos_start.position.z
        angles = eulerof(pos_end.orientation) - eulerof(pos_start.orientation)
        ret.angular.x = angles[0]
        ret.angular.y = angles[1]
        ret.angular.z = angles[2]
        return ret
    def __init__(self, publisher):
        self.pub = publisher
        self.current = genTwistMsg([0,0,0])
        self.recorded = {}
        self.diff = {}
    def callback(self, detections):
        self.diff.clear()
        # collect diff into self.diff
        # newly appear: False, None
        # previously recorded: True, diff
        for det in detections.detections:
            id = det.id
            pose = det.pose
            if id in self.recorded:
                diff = LocatorNode.pose_diff(self.recorded[id], pose)
                self.diff[id] = (True, diff)
            else:
                self.diff[id] = (False, None)
            self.recorded[id] = pose
        # remove unseen tags
        for id in list(self.recorded.keys()):
            if id not in self.diff:
                del self.recorded[id]
        # write log for now
        log = []
        for k,(e, v) in self.diff.items():
            if e:
                log.append('#%s\tdx=%s\tdy=%s\tdz=%s\tax=%s\tay=%s\taz=%s' % (
                    k, v.linear.x, v.linear.y, v.linear.z,
                    v.angular.x, v.angular.y, v.angular.z
                ))
            else:
                log.append('$%s\tNew' % k)
        rospy.loginfo('\n'.join(log))


def main():
    pub_twist = rospy.Publisher("/location", Twist, queue_size=1)
    loc = LocatorNode(pub_twist)
    rospy.init_node('april_locator')
    rospy.Subscriber('/apriltag_detection_array', AprilTagDetectionArray, loc.callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()
