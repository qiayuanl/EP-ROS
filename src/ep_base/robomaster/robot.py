# -- coding: utf-8 --
from __future__ import absolute_import

from robomaster.connection import Connection, ConnectionType
from robomaster.module import AI
from robomaster.module import Arm
from robomaster.module import Armor
from robomaster.module import Blaster
from robomaster.module import Camera
from robomaster.module import Chassis
from robomaster.module import Gimbal
from robomaster.module import Gripper
from robomaster.module import LED
from robomaster.module import Microphone
from robomaster.module import Sound
from robomaster.module import Status, RobotModeType


class Robot(object):
    def __init__(self, connection_type=ConnectionType.WIFI_DIRECT, robot_ip=u''):  # ConnectionType.WIFI_NETWORKING
        self.connection = Connection(connection_type=connection_type, robot_ip=robot_ip)  # robot_ip='10.34.13.5'
        self._setup()

    def _setup(self):
        ret = self.connection.open()
        if ret[0] is True:
            info = (u'连接成功\n'
                    u'机器人IP地址: {}\n'
                    u'当前设备IP地址: {}'
                    .format(self.connection.robot_ip, self.connection.device_ip))
            print info
        else:
            print u'连接失败, {}'.format(ret[1])
            return

        self.camera = Camera(self.connection)
        ret = self.connection.command_on()
        self.status = Status(self.connection)
        self.status.mode = RobotModeType.FREE
        print u"机器人剩余电量: %d%%" % ((int)(self.status.get_battery_power()))
        self.connection.command(u'robot mode free')
        self.led = LED(self.connection)
        self.chassis = Chassis(self.connection)
        self.gimbal = Gimbal(self.connection)
        self.blaster = Blaster(self.connection)
        self.microphone = Microphone(self.connection)

        self.ai = AI(self.connection)
        self.gripper = Gripper(self.connection)
        self.arm = Arm(self.connection)
        self.armor = Armor(self.connection)
        self.sound = Sound(self.connection)

    def close(self):
        if self.camera.on:
            self.camera.close()
        if self.microphone.on:
            self.microphone.close()
        # self.ai.subscribe_close()
        self.chassis.subscribe_close()
        self.connection.close()
