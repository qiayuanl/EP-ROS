#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import absolute_import

import math
import rospkg
import signal
import time

import rospy
import tf
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion

from robomaster import Robot
from robomaster.connection import ConnectionType
from robomaster.module import RobotChassisPushAttrType
from robomaster.module import RobotModeType


class EpNode:
    def __init__(self):
        # Robot hardware
        self.robot = Robot(connection_type=ConnectionType.WIFI_NETWORKING, robot_ip='192.168.0.135')
        self.robot.status.mode = RobotModeType.FREE
        self.robot.camera.open(height=360, width=640)
        self.robot.chassis.subscribe_open([RobotChassisPushAttrType.ATTITUDE, RobotChassisPushAttrType.POSITION],
            [50, 50])
        self.robot.led.color = (0, 0, 0)
        self.robot.arm.y = 80
        self.robot.arm.x = 180
        self.robot.gripper.open = True

        # ROS pub/sub
        self.image_pub = rospy.Publisher("/ep_image", Image, queue_size=10)
        self.camInfo_pub = rospy.Publisher("/ep_camInfo", CameraInfo, queue_size=10, latch=True)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.br = tf.TransformBroadcaster()
        self.bridge = CvBridge()
        self.make_camera_info()

        self.robot.camera.observe(self.image_cb, 'image')
        self.robot.chassis.observe(self.chassis_cb, 'subscribe_data')
        time.sleep(1)

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
            "base_link",
            "odom")

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
        # print "wheel speed: ", w0, w1, w2, w3
        self.robot.connection.command('chassis wheel w1 %f w2 %f w3 %f w4 %f' % (w0, w1, w2, w3))

    def cmd_vel_callback(self, msg):
        self.move_with_wheel_speed(msg.linear.x, msg.linear.y, -msg.angular.z * 57.29578)


def ep_exit(signum, frame):
    EpNode.is_alive = False


if __name__ == '__main__':
    rospy.init_node('ep_base', anonymous=True)
    ep_node = EpNode()
    signal.signal(signal.SIGINT, ep_exit)
    signal.signal(signal.SIGTERM, ep_exit)
    rospy.spin()
    ep_node.robot.close()
