# coding=utf-8
from __future__ import absolute_import

import sys

sys.path.append(u'..')
from robomaster import Robot
from robomaster.connection import ConnectionType  # 连接方式的枚举类型
from robomaster.module import RobotModeType  # 机器人运动模式的枚举类型

robot = Robot(connection_type=ConnectionType.WIFI_NETWORKING, robot_ip='192.168.1.102')

# 运动模式更换示例 begin
robot.status.mode = RobotModeType.FREE

# 运动模式更换示例 end
while True:
    robot.arm.x = int(input('x'))
    robot.arm.y = int(input('y'))

time.sleep(1)

robot.close()
print
u'示例结束'
