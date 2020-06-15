#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import absolute_import

import math
import rospkg
import rospy
import signal
import tf
import threading
import time
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from robomaster import Robot
from robomaster.connection import ConnectionType
from robomaster.module import RobotChassisPushAttrType
from robomaster.module import RobotModeType
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class EP_Node:
    def __init__(self):
        self.robot = Robot(connection_type=ConnectionType.WIFI_NETWORKING, robot_ip='192.168.0.135')
        self.robot.status.mode = RobotModeType.FREE
        self.robot.camera.open(height=360, width=640)
        self.robot.chassis.subscribe_open([RobotChassisPushAttrType.ATTITUDE, RobotChassisPushAttrType.POSITION],
                                          [30, 30])

        self.robot.led.color = (0, 0, 0)
        time.sleep(2)
        self.robot.arm.y = 80
        self.robot.arm.x = 180
        self.robot.gripper.open = True
        self.image_pub = rospy.Publisher("/ep_image", Image, queue_size=10)
        self.camInfo_pub = rospy.Publisher("/ep_camInfo", CameraInfo, queue_size=10, latch=True)
        self.manual_point_pub = rospy.Publisher("/manual_point", PointStamped, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.item_pose_sub = rospy.Subscriber("/item_pose", PoseStamped, self.item_cali_pos_callback)
        self.item_vel_sub = rospy.Subscriber("/calib_vel", Twist, self.item_calib_vel_callback)

        self.br = tf.TransformBroadcaster()
        self.bridge = CvBridge()

        self.caminfomsg = self.make_camera_msg()
        self.is_alive = True
        self.image_pub_thread = threading.Thread(target=self.__image_pub_theading)
        self.image_pub_thread.start()

        # self.robot.camera.observe(self.image_cb,'image')
        # self.robot.chassis.observe(self.chassis_cb,'subscribe_data')

        self.axes = (-0.0, -0.0, 0.0, -0.0, -0.0, 0.0, 0.0, 0.0)
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.calib_vel = [0.0, 0.0, 0.0]
        self.calib_vel_lastframe = [0.0, 0.0, 0.0]
        self.item_position = [0.0, 0.0, 0.0]
        self.euler_rpy = [0.0, 0.0, 0.0]
        self.pos_yaw = 0.0

        self.manual_point_origin = [[0, -1.6], [1.6, -1.6], [0.8, -0.8], [1.6, 0]]
        self.wait_times_w = 0
        # FSM flag
        self.statusNum = 0
        self.planSuccess = False  # default false
        self.planStart = True
        self.scanStart = True  # default false
        self.scanSuccess = True
        self.calibSuccess = False  # default false
        self.gripSuccess = False
        self.releaseSuccess = False
        self.autoGrip = False
        self.manualContor = True
        self.gripper_times = 0
        time.sleep(1)

        # self.camInfo_pub.publish(caminfomsg)

    def __image_pub_theading(self):
        while self.is_alive:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.robot.camera.image, "bgr8"))
            # caminfomsg = self.make_camera_msg()
            self.camInfo_pub.publish(self.caminfomsg)
            # cv2.imshow(u"image", self.robot.camera.image)
            # cv2.waitKey(1)
            x = self.robot.chassis.subscribe_data.position.x
            y = self.robot.chassis.subscribe_data.position.y
            pitch = self.robot.chassis.subscribe_data.attitude.pitch / 57.3
            roll = self.robot.chassis.subscribe_data.attitude.roll / 57.3
            yaw = self.robot.chassis.subscribe_data.attitude.yaw / 57.3
            self.br.sendTransform((x, -y, 0),
                                  tf.transformations.quaternion_from_euler(0, -0, -yaw),
                                  rospy.Time.now(),
                                  "base_link",
                                  "odom")
            time.sleep(0.03)
        print('image publish theading exit')

    def to_point_stamped(self, origin):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "map"
        point.point.x = origin[0]
        point.point.y = origin[1]
        point.point.z = 0.0
        return point

    # 相机信息打包
    def make_camera_msg(self):
        camera_info_msg = CameraInfo()
        rospack = rospkg.RosPack()
        cam_file = open(rospack.get_path('ep_bringup') + '/config/head_camera.yaml')
        cam_data = yaml.safe_load(cam_file)

        camera_info_msg.header = rospy.Header()
        camera_info_msg.header.frame_id = cam_data["camera_name"]
        # camera_info_msg.header.stamp = rospy.Time.now()

        camera_info_msg.distortion_model = cam_data["distortion_model"]
        camera_info_msg.width = cam_data['image_width']
        camera_info_msg.height = cam_data['image_height']
        camera_info_msg.binning_x = 0
        camera_info_msg.binning_y = 0

        camera_info_msg.K = cam_data['camera_matrix']['data']
        camera_info_msg.D = cam_data['distortion_coefficients']['data']
        camera_info_msg.R = cam_data['rectification_matrix']['data']
        camera_info_msg.P = cam_data['projection_matrix']['data']

        return camera_info_msg

    def image_cb(self, change):
        time1 = time.time()
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.robot.camera.image, "bgr8"))
        print(time.time() - time1)
        # cv2.imshow(u"image", change[u'new'])
        # cv2.waitKey(30)

    def chassis_cb(self, change):
        x = robot.chassis.subscribe_data.position.x
        y = robot.chassis.subscribe_data.position.y
        pitch = robot.chassis.subscribe_data.attitude.pitch / 57.3
        roll = robot.chassis.subscribe_data.attitude.roll / 57.3
        yaw = robot.chassis.subscribe_data.attitude.yaw / 57.3
        self.br.sendTransform((x, -y, 0),
                              tf.transformations.quaternion_from_euler(0, -0, -yaw),
                              rospy.Time.now(),
                              "base_link",
                              "odom")
        # 麦轮解算，提高控制精度为0.07m/s

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

    # 导航速度回调
    def cmd_vel_callback(self, msg):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = -msg.angular.z * 57.29578
        if msg.linear.z == 1:
            self.planSuccess = True
        else:
            self.planSuccess = False
        print("cmd_vel:", self.cmd_vel)

    # 物块对位的位置回调
    def item_cali_pos_callback(self, msg):
        if self.scanStart is True:
            # 判断id
            self.item_position[0] = msg.pose.position.x
            self.item_position[1] = -msg.pose.position.y
            self.item_position[2] = msg.pose.position.z
            (item_r, item_p, item_y) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,
                                                                                 msg.pose.orientation.y,
                                                                                 msg.pose.orientation.z,
                                                                                 msg.pose.orientation.w])
            self.euler_rpy[0] = item_r * 57.29578
            self.euler_rpy[1] = item_p * 57.29578
            self.euler_rpy[2] = -(item_y * 57.29578 + 90)
            if abs(self.euler_rpy[2]) <= 3:
                self.scanSuccess = True
            print("pos_x: ", self.item_position[0], "pos_y: ", self.item_position[1], "pos_yaw: ", self.euler_rpy[2])

        # 物块对位的速度回调

    def item_calib_vel_callback(self, msg):
        if self.scanSuccess is True:
            self.calib_vel[0] = msg.linear.x
            self.calib_vel[1] = -msg.linear.y
            self.calib_vel[2] = -msg.angular.z * 57.3
            if msg.linear.z == 1:
                self.calibSuccess = True
            else:
                self.calibSuccess = False
        # print "calib_vel:", self.calib_vel

    # 状态机处理流程
    def fsm_process(self):

        if self.statusNum == 0:
            self.fsm_path_plan()
        elif self.statusNum == 1:
            self.fsm_item_calib()
        elif self.statusNum == 2:
            self.fsm_item_grip()
        elif self.statusNum == 3:
            self.fsm_item_trans()
        elif self.statusNum == 4:
            self.fsm_item_release()

    def fsm_pub_goal_point_start(self):
        for i in self.manual_point_origin:
            self.manual_point_pub.publish(self.to_point_stamped(i))
            print('send point :', i)
            time.sleep(0.2)
        self.statusNum += 1

    def fsm_pub_goal_point_back(self):
        for i in range(0, 4):
            self.manual_point_pub.publish(self.to_point_stamped(self.manual_point_origin[3 - i]))
            print('send point :', i)
            time.sleep(0.2)
        self.statusNum += 1
        # 导航

    def fsm_path_plan(self):

        if self.planSuccess is True:
            print("Plan success --- ready to scan")
            self.planSuccess = False
            self.scanSuccess = False
            self.statusNum += 1
            self.move_with_wheel_speed(0, 0, 0)
        else:
            self.move_with_wheel_speed(self.cmd_vel[0], self.cmd_vel[1], self.cmd_vel[2])

    # 物块对准
    def fsm_item_calib(self):
        while self.scanSuccess is False:
            self.scanStart = True
            self.scan_item()
        self.robot.chassis.speed.x = 0
        self.robot.chassis.speed.y = 0
        self.robot.chassis.speed.w = 0  # 扫描成功后，底盘置0
        self.scanStart = False
        print("Scan success --- ready to calib")
        while self.calibSuccess is False:
            if abs(self.calib_vel[0] - 0) < 1.0e-8 and abs(self.calib_vel[1] - 0) < 1.0e-8 and abs(
                    self.calib_vel[2] - 0) < 1.0e-8:
                self.move_with_wheel_speed(0, 0, 0)
                time.sleep(0.033)
            else:
                # TODO:替换为麦轮解算

                self.move_with_wheel_speed(self.calib_vel[0], self.calib_vel[1], self.calib_vel[2])

                time.sleep(0.033)
        self.move_with_wheel_speed(0, 0, 0)
        print("Calib success --- ready to grip")
        self.calibSuccess = True
        self.statusNum += 1

    # 扫描四周，寻找物块
    def scan_item(self):
        self.robot.chassis.speed.x = 0
        self.robot.chassis.speed.y = 0
        self.robot.chassis.speed.w = 15
        time.sleep(0.5)

    # 物块抓取
    def fsm_item_grip(self):
        self.robot.arm.x = 200
        self.robot.arm.y = -80
        time.sleep(3)
        self.robot.gripper.open = False
        time.sleep(3)
        self.robot.arm.x = 180
        self.robot.arm.y = 80
        self.robot.gripper.open = False
        time.sleep(3)
        print("Grip success --- ready to transport")
        self.statusNum += 1

    # 物块运输
    def fsm_item_trans(self):
        if self.planSuccess is True:
            print("Transport success --- ready to release")
            self.planSuccess = False
            self.statusNum += 1
            self.move_with_wheel_speed(0, 0, 0)
        else:
            self.move_with_wheel_speed(self.cmd_vel[0], self.cmd_vel[1], self.cmd_vel[2])

    # 物块释放
    def fsm_item_release(self):
        self.robot.arm.x = 180
        self.robot.arm.y = 80
        self.robot.gripper.open = True
        print("Release success --- ready to plan next")
        self.gripper_times += 1
        # if self.gripper_times>2-1:
        #     self.statusNum += 1
        # else:
        #     self.statusNum = 0 
        time.sleep(2)


def exit(signum, frame):
    EP_Node.is_alive = False


if __name__ == '__main__':
    rospy.init_node('S1_test', anonymous=True)
    EP_node = EP_Node()
    rate = rospy.Rate(30)
    signal.signal(signal.SIGINT, exit)
    signal.signal(signal.SIGTERM, exit)
    while not rospy.is_shutdown():
        EP_node.fsm_process()
        rate.sleep()
    EP_node.robot.close()
