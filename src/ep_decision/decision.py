#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import absolute_import

from enum import Enum

import geometry_msgs.msg
import rospy
import tf
from ep_base.msg import Gripper


class StateType(Enum):
    MOVETO = 1
    CALIBRATE = 2
    PICK = 3
    PLACE = 4


class Fsm(object):
    def __init__(self):
        self.vel_pub = rospy.Publisher("/vel_cmd", geometry_msgs.msg.Twist, queue_size=10)
        self.gripper_pub = rospy.Publisher("/gripper", Gripper, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.ls = tf.TransformListener()
        self.state = None
        self.base_pos = None
        self.base_ori = None
        self.gripper_pos = None

    def run(self):

        if self.state == StateType.MOVETO:
            self.move_to()
        if self.state == StateType.CALIBRATE:
            self.calibrate()
        if self.state == StateType.PICK:
            self.pick()
        if self.state == StateType.PLACE:
            self.place()

    def update(self):  #
        try:
            (self.gripper_pos, rot) = self.ls.lookupTransform('/base_link', '/gripper', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        try:
            (self.base_pos, rot) = self.ls.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        self.base_ori = tf.transformations.euler_from_quaternion(rot)

    def move_to(self):
        pass

    def calibrate(self):
        pass

    def pick(self):
        pass

    def place(self):
        pass


if __name__ == '__main__':
    rospy.init_node('ep_decision', anonymous=True)
    rospy.spin()
