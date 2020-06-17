# coding=utf8
from __future__ import absolute_import

import sys
import threading
from enum import Enum

import traitlets
from traitlets.config.configurable import Configurable


class RobotChassisPushAttrType(Enum):
    POSITION = 1
    ATTITUDE = 2
    STATUS = 3


class Attitude(object):
    pitch = 0
    roll = 0
    yaw = 0


class Position(object):
    x = 0
    y = 0


class SubscribeData(object):
    attitude = Attitude()
    position = Position()
    status = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


class Chassis(Configurable):
    subscribe_data = traitlets.Any()

    def __init__(self, connection):
        super(Chassis, self).__init__()  # initializes traitlets
        self.connection = connection
        self.data_analysis_thread = threading.Thread(target=self.__chassis_push_analysis_task)
        self.get_push_data_ready = threading.Event()
        self.subscribe_data = None
        self.push_data = {}
        self.push_freq = 50
        self.position_on = False
        self.attitude_on = False
        self.status_on = False
        self.position_freq = 50
        self.attitude_freq = 50
        self.status_freq = 10

    def subscribe_open(self, push_type=[], freq=[]):
        if len(push_type) != len(freq):
            print u"选择的推送类型与对应的频率数量不相同"
            return
        for i in xrange(0, len(push_type)):
            if push_type[i] == RobotChassisPushAttrType.ATTITUDE:
                self.attitude_on = True
                self.attitude_freq = freq[i]
            elif push_type[i] == RobotChassisPushAttrType.POSITION:
                self.position_on = True
                self.position_freq = freq[i]
            elif push_type[i] == RobotChassisPushAttrType.STATUS:
                self.status_on = True
                self.status_freq = freq[i]

        self.connection.push_on = True
        self.data_analysis_thread.start()
        self.get_push_data_ready.wait()

    def subscribe_close(self, push_type=[]):
        attr = u''
        if push_type:
            for i in xrange(0, len(push_type)):
                if push_type[i] == RobotChassisPushAttrType.ATTITUDE and self.attitude_on is True:
                    self.attitude_on = False
                    attr += u' attitude off'
                elif push_type[i] == RobotChassisPushAttrType.POSITION and self.position_on is True:
                    self.position_on = False
                    attr += u' position off'
                elif push_type[i] == RobotChassisPushAttrType.STATUS and self.status_on is True:
                    self.status_on = False
                    attr += u' position off'
        else:
            if self.attitude_on is True:
                self.attitude_on = False
                attr += u' attitude off'
            elif self.position_on is True:
                self.position_on = False
                attr += u' position off'
            elif self.status_on is True:
                self.status_on = False
                attr += u' status off'
        if attr:
            self.connection.command(u'chassis push' + attr)
        print u'底盘推送:关闭'

    def __chassis_push_analysis_task(self):
        print u'开启底盘推送： ',
        sys.stdout.write(u'')
        send_data = u''
        if self.position_on:
            send_data += u' position on pfreq ' + unicode(self.position_freq)
        if self.attitude_on:
            send_data += u' attitude on afreq ' + unicode(self.attitude_freq)
        if self.status_on:
            send_data += u' status on sfreq ' + unicode(self.status_freq)
        self.connection.command(u"chassis push" + send_data)
        self.connection.start_push_recv()
        print u'成功'

        while self.status_on or self.position_on or self.attitude_on:
            buff = self.connection.recv_push_data(u'chassis')
            if buff:
                self.get_push_data_ready.set()
                self.subscribe_data = SubscribeData()
                buf_list = buff.split(u' ')
                # print(buf_list)
                if u"position" in buf_list:
                    self.subscribe_data.position.x = float(buf_list[buf_list.index(u'position') + 1])
                    self.subscribe_data.position.y = float(buf_list[buf_list.index(u'position') + 2])
                    # print('chassis x=%f,y=%f' % (self.subscribe_data.position.x, self.subscribe_data.position.y))

                if u"attitude" in buf_list:
                    self.subscribe_data.attitude.pitch = float(buf_list[buf_list.index(u'attitude') + 1])
                    self.subscribe_data.attitude.roll = float(buf_list[buf_list.index(u'attitude') + 2])
                    self.subscribe_data.attitude.yaw = float(buf_list[buf_list.index(u'attitude') + 3])
                    # print('chassis pitch=%f,roll=%f,yaw=%f' % (
                    #     self.subscribe_data.attitude.pitch, self.subscribe_data.attitude.roll,
                    #     self.subscribe_data.attitude.yaw))
                if u"status" in buf_list:
                    self.subscribe_data.status = buf_list[(buf_list.index(u'status') + 1):]

                    # self.connection.command('chassis push position off attitude off')
        self.connection.command(u'chassis push position off')
        # self.connection.stop_video_recv()
