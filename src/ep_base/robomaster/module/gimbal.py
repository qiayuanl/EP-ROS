# coding=utf8
from __future__ import absolute_import

import sys
import threading
import traitlets
from traitlets.config.configurable import Configurable


class SubscribeData(object):
    pitch = 0
    yaw = 0


class Speed(Configurable):
    pitch = traitlets.Float()
    yaw = traitlets.Float()

    def __init__(self, connection):
        super(Speed, self).__init__()  # initializes traitlets
        self.connection = connection

    def reset(self):
        self.pitch = 0.0
        self.yaw = 0.0

    @traitlets.observe(u'pitch', u'yaw')
    def __observe_speed(self, change):
        self.connection.command(u'gimbal speed p %f y %f'
                                % (self.pitch, self.yaw))


class Pose(Configurable):
    pitch = traitlets.Float()
    yaw = traitlets.Float()

    def __init__(self, connection):
        super(Pose, self).__init__()  # initializes traitlets
        self.connection = connection

    def reset(self):
        self.pitch = 0.0
        self.yaw = 0.0

    @traitlets.observe(u'pitch', u'yaw')
    def __observe_speed(self, change):
        self.connection.command(u'gimbal moveto p %f y %f wait_for_complete false'
                                % (self.pitch, self.yaw))


class Gimbal(Configurable):
    speed = traitlets.Instance(Speed)
    pose = traitlets.Instance(Pose)

    def __init__(self, connection):
        super(Gimbal, self).__init__()  # initializes traitlets
        self.speed = Speed(connection)
        self.pose = Pose(connection)
        self.connection = connection

        self.data_analysis_thread = threading.Thread(target=self.__gimbal_push_analysis_task)
        self.get_push_data_ready = threading.Event()

        self.on = False
        self.subscribe_data = SubscribeData()
        self.push_freq = 50

    def subscribe_open(self, push_freq=50):
        self.push_freq = push_freq
        if self.on:
            print
            u'成功开启云台推送'
            return
        self.on = True
        self.connection.push_on = True
        self.data_analysis_thread.start()
        self.get_push_data_ready.wait()

    def subscribe_close(self):
        if self.on:
            self.on = False
            self.connection.command(u'gimbal push attitude off')
            print
            u'云台推送:关闭'
        else:
            self.on = False

    def __gimbal_push_analysis_task(self):
        print
        u'开启云台姿态推送： ',;
        sys.stdout.write(u'')
        ret = self.connection.command(u'gimbal push attitude on %d' % (self.push_freq))
        print
        u'成功'
        self.connection.start_push_recv()

        while True:
            buff = self.connection.recv_push_data(u'gimbal')
            if buff:
                self.get_push_data_ready.set()
                buf_list = buff.split(u' ')
                # print(buf_list)
                self.subscribe_data.pitch = float(buf_list[3])
                self.subscribe_data.yaw = float(buf_list[4])
                # print('pitch=%f,yaw=%f' % (self.subscribe_data.pitch, self.subscribe_data.yaw))
        self.connection.command(u'gimbal push attitude off')
