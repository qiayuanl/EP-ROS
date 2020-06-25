#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import absolute_import

import math
import rospkg
import threading
import time

import geometry_msgs.msg
import rospy
import tf
import yaml
from cv_bridge import CvBridge
from ep_base.msg import Gripper
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from robomaster import Robot
from robomaster.connection import ConnectionType
from robomaster.module import RobotChassisPushAttrType
from robomaster.module import RobotModeType


class EpNode:
    def __init__(self):
        # ROS Param
        self.is_alive = True
        ip = rospy.get_param('ep_ip', '192.168.0.135')
        chassis_freq = rospy.get_param('chassis_freq', 50)
        self.gripper_freq = rospy.get_param('gripper_freq', 10)
        # Robot hardware
        self.robot = Robot(connection_type=ConnectionType.WIFI_NETWORKING, robot_ip=ip)
        self.robot.status.mode = RobotModeType.FREE
        self.robot.camera.open(height=360, width=640)
        self.robot.chassis.subscribe_open([RobotChassisPushAttrType.ATTITUDE, RobotChassisPushAttrType.POSITION],
                                          [chassis_freq, chassis_freq])
        self.robot.led.color = (0, 0, 0)
        self.robot.arm.reset()
        # self.robot.arm.x = 70
        # self.robot.arm.y = 40

        self.robot.gripper.open = True

        # ROS pub/sub
        self.image_pub = rospy.Publisher("/ep_camera/image_raw", Image, queue_size=10)
        self.camInfo_pub = rospy.Publisher("/ep_camera/camera_info", CameraInfo, queue_size=10, latch=True)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", geometry_msgs.msg.Twist, self.cmd_vel_cb, queue_size=1)
        self.cmd_grip_sub = rospy.Subscriber("/gripper", Gripper, self.cmd_grip_cb, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.ls = tf.TransformListener()
        self.bridge = CvBridge()
        self.make_camera_info()

        self.grip_pub_thread = threading.Thread(target=self.grip_pub_cb)
        self.grip_pub_thread.start()

        self.robot.camera.observe(self.image_cb, 'image')
        self.robot.chassis.observe(self.chassis_cb, 'subscribe_data')
        time.sleep(1)
        self.br.sendTransform((0.1, 0, 0),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "odom",
                              "map")

    # Camera Info Setup
    def make_camera_info(self):
        self.camera_info = CameraInfo()
        ros_pack = rospkg.RosPack()
        cam_file = open(ros_pack.get_path('ep_bringup') + '/config/head_camera.yaml')
        cam_data = yaml.safe_load(cam_file)

        self.camera_info.header = rospy.Header()
        self.camera_info.header.frame_id = cam_data["camera_name"]

        self.camera_info.distortion_model = cam_data["distortion_model"]
        self.camera_info.width = cam_data['image_width']
        self.camera_info.height = cam_data['image_height']
        self.camera_info.binning_x = 0
        self.camera_info.binning_y = 0

        self.camera_info.K = cam_data['camera_matrix']['data']
        self.camera_info.D = cam_data['distortion_coefficients']['data']
        self.camera_info.R = cam_data['rectification_matrix']['data']
        self.camera_info.P = cam_data['projection_matrix']['data']

    def image_cb(self, change):
        # cv2.imshow(u"image", change[u'new'])
        # cv2.waitKey(1)
        self.camInfo_pub.publish(self.camera_info)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.robot.camera.image, "bgr8"))

    def chassis_cb(self, change):
        x = self.robot.chassis.subscribe_data.position.x
        y = self.robot.chassis.subscribe_data.position.y
        # pitch = self.robot.chassis.subscribe_data.attitude.pitch / 57.3
        # roll = self.robot.chassis.subscribe_data.attitude.roll / 57.3
        yaw = self.robot.chassis.subscribe_data.attitude.yaw / 57.3
        self.br.sendTransform((x, -y, 0),
                              tf.transformations.quaternion_from_euler(0, -0, -yaw),
                              rospy.Time.now(),
                              "/base_link",
                              "/odom")
        try:
            (trans, rot) = self.ls.lookupTransform('/map', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        self.br.sendTransform(trans, rot, rospy.Time.now(), "/odom", "/map")

    def move_with_wheel_speed(self, x=0.0, y=0.0, yaw=0.0):
        yaw = -yaw / 57.3
        a = 0.10
        b = 0.10
        r = 0.05
        pi = math.pi
        w0 = (x - y + yaw * (a + b)) * (30 / pi) / r
        w1 = (x + y - yaw * (a + b)) * (30 / pi) / r
        w2 = (x - y - yaw * (a + b)) * (30 / pi) / r
        w3 = (x + y + yaw * (a + b)) * (30 / pi) / r
        self.robot.connection.command('chassis wheel w1 %f w2 %f w3 %f w4 %f' % (w0, w1, w2, w3))

    def cmd_vel_cb(self, msg):
        self.move_with_wheel_speed(msg.linear.x, msg.linear.y, -msg.angular.z * 57.29578)

    def cmd_grip_cb(self, msg):
        self.robot.arm.x = msg.x
        self.robot.arm.y = msg.z
        self.robot.gripper.open = msg.grip

    def grip_pub_cb(self):
        while self.is_alive:
            ret, now_pos = self.robot.connection.command(u'robotic_arm position ?')
            x = float(now_pos.split(u' ')[0]) / 1000
            z = float(now_pos.split(u' ')[1]) / 1000
            self.br.sendTransform((x, 0, z),
                                  tf.transformations.quaternion_from_euler(0, 0, 0),
                                  rospy.Time.now(),
                                  "gripper", "base_link")
            time.sleep(1. / self.gripper_freq)
        print('gripper publish thread exit')


def ep_exit():
    ep_node.is_alive = False
    ep_node.robot.camera.unobserve(ep_node.image_cb, 'image')
    ep_node.robot.chassis.unobserve(ep_node.chassis_cb, 'subscribe_data')


if __name__ == '__main__':
    rospy.init_node('ep_base', anonymous=True)
    ep_node = EpNode()
    # signal.signal(signal.SIGINT, ep_exit)
    # signal.signal(signal.SIGTERM, ep_exit)
    rospy.spin()
    ep_exit()
    ep_node.robot.close()
