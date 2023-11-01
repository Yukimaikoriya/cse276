#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from april_detection.msg import AprilTagDetectionArray
import tf.transformations as tft
import tf
import numpy as np


def point(x, y, z):
    ret = Point()
    ret.x = x
    ret.y = y
    ret.z = z
    return ret
def m2a(m):
    t = list(tft.translation_from_matrix(m))
    a = tft.euler_from_matrix(m)
    t.extend(a)
    return t
def a2m(a):
    mt = tft.translation_matrix(a[:3])
    ma = tft.euler_matrix(a[3], a[4], a[5])
    return tft.concatenate_matrices(mt, ma)
def pose2m(pose):
    mt = tft.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
    ma = tft.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    return tft.concatenate_matrices(mt, ma)


class Tag:
    N_CALIBRATE = 5
    CALI_THRESH = 0.01
    @staticmethod
    def nameOf(x):
        return 'tag%s' % x
    def __init__(self, id):
        self.id = id
        self.name = Tag.nameOf(id)
        self.calibration = []
        self.isCalibrated = False
        self.tf_from_world = None
        rospy.loginfo('Calibration started for %s' % self.name)
        # tag.pose + params = rf
    def try_calibrate(self):
        # if the differences of N_CALIBRATE consecutive frames are small enough
        # then use the average of the frames
        data = []
        for t in self.calibration:
            data.append(t)
        data = np.array(data)
        df = np.diff(data, axis=0)
        if np.max(df) > Tag.CALI_THRESH:
            return False
        avg = np.average(data, axis=0)
        self.tf_from_world = avg
        self.isCalibrated = True
        rospy.loginfo('Calibration done for %s' % self.name)
        return True
    def calibrate(self, det, m_w2c):
        m_c2t = pose2m(det.pose)
        self.calibration.append(tft.concatenate_matrices(m_w2c, m_c2t))
        if len(self.calibration) >= Tag.N_CALIBRATE:
            if not self.try_calibrate():
                self.calibration = self.calibration[1:]
    def estimate(self, det):
        if not self.isCalibrated:
            raise Exception('%s must be calibrated before estimation!' % self.name)
        c2t = pose2m(det.pose)
        w2t = self.tf_from_world
        w2c = tft.concatenate_matrices(w2t, tft.inverse_matrix(c2t))
        return w2c


class LocatorNode:
    
    def __init__(self, publisher):
        self.pub = publisher
        self.tf = tf.Transformer(True)
        self.tags = {}
        self.last_known = a2m(np.zeros(6))

    def callback(self, detections):
        # Step 2: Estimate tf world -> camera
        m_tfs = []
        for det in detections.detections:
            if det.id not in self.tags:
                self.tags[det.id] = Tag(det.id)
            tag = self.tags[det.id]
            if tag.isCalibrated:
                m_tfs.append(tag.estimate(det))
        m_w2c = self.last_known
        if m_tfs:
            m_w2c = np.average(np.array(m_tfs), axis=0)
        self.last_known = m_w2c

        # Step 3: publish tf world -> camera
        a_w2c = m2a(m_w2c)
        self.pub.publish(point(a_w2c[2], -a_w2c[0], a_w2c[4])) # x, y, theta

        # Step 4: do calibration for the uncalibrated
        for det in detections.detections:
            tag = self.tags[det.id]
            if not tag.isCalibrated:
                tag.calibrate(det, m_w2c)
        rospy.loginfo('%s' % m2a(m_w2c))


def main():
    pub_twist = rospy.Publisher("/rf_robot", Point, queue_size=1)
    loc = LocatorNode(pub_twist)
    rospy.init_node('april_locator')
    rospy.Subscriber('/apriltag_detection_array', AprilTagDetectionArray, loc.callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()
