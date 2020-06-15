#! python3


from __future__ import absolute_import
from __future__ import division

import PID
import time
from robomaster import Robot
from robomaster.connection import ConnectionType
from robomaster.module import RobotAIPushAttrType, RobotAIAttrType
from robomaster.module import RobotModeType


def line_data():
    # eg: line_data[0][1][2] 含义为第0条线，第1个点，切线角数据
    # eg：line_data[0][1][？] ？从0~3依次代表，点x坐标，点y坐标，点切线角度，点对应曲线的曲率
    line_data = robot.ai.subscribe_data[u'line']
    if line_data:  # 如果检测到线
        temp_total_angle = 0
        for cout in xrange(3, 6):
            temp_total_angle += line_data[0][cout][2]  # 第0根线的第3到第6个点的切线角数据累积均值
        # print([110 - abs(temp_total_angle / 3), line_data[0][1][1], temp_total_angle / 3])
        return [110 - abs(temp_total_angle / 3), line_data[0][1][0], temp_total_angle / 3]
    else:
        return False


robot = Robot(connection_type=ConnectionType.USB_DIRECT)
# 初始化PID参数
pid_angle = PID.PID(5, 0, 0)
pid_x = PID.PID(0.004, 0, 0)
pid_y = PID.PID(2.5, 0, 0)
robot.gimbal.pose.yaw = 0
time.sleep(1)
# 云台跟随底盘模式
robot.status.mode = RobotModeType.CHASSIS_LEAD
# print(robot.connection.command('robot mode ?'))
time.sleep(3)
robot.gimbal.pose.pitch = -20
robot.ai.subscribe_attribute([RobotAIAttrType.LINE_BLUE])
robot.ai.subscribe_open([RobotAIPushAttrType.LINE, RobotAIPushAttrType.MARKER])
while 1:
    buf_data = line_data()
    if buf_data != False:
        print
        u"success"
        pid_angle.update(0, buf_data[2])
        pid_x.update(0, buf_data[0])
        pid_y.update(0.5, buf_data[1])
        # 去掉了前进
        # robot.chassis.speed.x = pid_x.output
        robot.chassis.speed.y = pid_y.output
        robot.chassis.speed.w = pid_angle.output
        # robot.chassis.speed.w = 0
    else:
        robot.chassis.speed.x = 0
        robot.chassis.speed.y = 0
        robot.chassis.speed.w = 0
    time.sleep(0.01)
