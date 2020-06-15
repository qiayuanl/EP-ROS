# coding=utf8
from __future__ import absolute_import

import numpy
import sys
import threading
from enum import Enum
from traitlets.config.configurable import Configurable


class RobotAIPushAttrType(Enum):
    PERSON = 1
    POSE = 2
    LINE = 3
    MARKER = 4
    ROBOT = 5


class RobotAIPoseIDType(Enum):
    V_POSE = 4
    INVERTED_V_POSE = 5
    SHOT_POSE = 6


class RobotAIMarkerIDType(Enum):
    STOP = 1
    TURN_LEFT = 4
    TURN_RIGHT = 5
    FORWARD = 6
    RED_HEART = 8


class RobotAIAttrType(Enum):
    LINE_RED = 1
    LINE_BLUE = 2
    LINE_GREEN = 3
    MAEKER_RED = 4
    MAEKER_BLUE = 5
    MAEKER_DICT = 6


class AI(Configurable):
    def __init__(self, connection):
        self.connection = connection
        self.data_analysis_thread = threading.Thread(target=self.__AI_push_analysis_task)
        self.get_push_data_ready = threading.Event()

        self.subscribe_data = {u'people': [],
                               u'pose': [],
                               u'marker': [],
                               u'line': [],
                               u'robot': []}
        self.AI_A_group = u''
        self.AI_B_group = u''

    def subscribe_open(self, push_type=[]):
        for i in xrange(len(push_type)):
            if push_type[i] == RobotAIPushAttrType.PERSON:
                self.AI_A_group = u' people on'
            elif push_type[i] == RobotAIPushAttrType.POSE in push_type:
                self.AI_A_group = u' pose on'
            elif push_type[i] == RobotAIPushAttrType.MARKER in push_type:
                self.AI_A_group = u' marker on'
            elif push_type[i] == RobotAIPushAttrType.ROBOT in push_type:
                self.AI_A_group = u' robot on'
            elif push_type[i] == RobotAIPushAttrType.LINE:
                self.AI_B_group = u' line on'

        self.connection.push_on = True
        self.data_analysis_thread.start()
        self.get_push_data_ready.wait()

    def subscribe_close(self):
        if self.AI_A_group or self.AI_B_group:
            self.AI_A_group.replace(u'on', u'off')
            self.AI_B_group.replace(u'on', u'off')
            ret = self.connection.command(u"AI push" + self.AI_A_group + self.AI_B_group)
            print
            u'AI推送:关闭'
            self.AI_A_group = u""
            self.AI_B_group = u""
            self.data_analysis_thread.join()

    def subscribe_attribute(self, attr, dist=0):
        for i in attr:
            if i == RobotAIAttrType.LINE_BLUE:
                self.connection.command(u"AI attribute line_color blue")
            elif i == RobotAIAttrType.LINE_RED:
                self.connection.command(u"AI attribute line_color red")
            elif i == RobotAIAttrType.LINE_GREEN:
                self.connection.command(u"AI attribute line_color green")
            elif i == RobotAIAttrType.MAEKER_BLUE:
                self.connection.command(u"AI attribute marker_color blue")
            elif i == RobotAIAttrType.MAEKER_RED:
                self.connection.command(u"AI attribute marker_color red")
            elif i == RobotAIAttrType.MAEKER_DICT:
                if dist > 0.5 and dist < 3:
                    self.connection.command(u"AI attribute marker_dist " + unicode(dist))
                else:
                    print
                    u"识别距离不在有效范围内"

    def __AI_push_analysis_task(self):
        print
        u'开启AI推送： ',;
        sys.stdout.write(u'')
        ret = self.connection.command(u"AI push" + self.AI_A_group + self.AI_B_group)
        print
        u'成功' + ret[1]
        self.connection.start_push_recv()

        while self.AI_A_group or self.AI_B_group:
            buff = self.connection.recv_push_data(u'AI')
            if buff:

                buf_list = buff.split(u' ')
                # print(buf_list)
                if u"people" in buf_list:
                    data = []
                    if int(buf_list[buf_list.index(u'people') + 1]) > 0:
                        for i in xrange(buf_list.index(u'people') + 2, len(buf_list), 4):
                            data.append([buf_list[i], buf_list[i + 1], buf_list[i + 2], buf_list[i + 3]])
                            self.subscribe_data[u'people'] = data
                            # print(self.subscribe_data['people'])

                if u"pose" in buf_list:
                    data = []
                    if int(buf_list[buf_list.index(u'pose') + 1]) > 0:
                        for i in xrange(buf_list.index(u'pose') + 2, len(buf_list), 5):
                            data.append(
                                [buf_list[i], buf_list[i + 1], buf_list[i + 2], buf_list[i + 3], buf_list[i + 4]])
                            self.subscribe_data[u'pose'] = data
                            # print(self.subscribe_data['pose'])
                if u"marker" in buf_list:
                    data = []
                    marker_num = int(buf_list[buf_list.index(u'marker') + 1])
                    if marker_num > 0:
                        for i in xrange(buf_list.index(u'marker') + 2, len(buf_list)):
                            data.append(float(buf_list[i]))
                        np_data = numpy.array(data).reshape(marker_num, 5)
                        self.subscribe_data[u'marker'] = np_data.tolist()
                        # print(self.subscribe_data['marker'])
                if u"line" in buf_list:
                    data = []
                    line_num = int(buf_list[buf_list.index(u'line') + 1])
                    if line_num > 0:
                        for i in xrange(buf_list.index(u'line') + 2, len(buf_list)):
                            data.append(float(buf_list[i]))
                        np_data = numpy.array(data).reshape(line_num, 10, 4)
                        self.subscribe_data[u'line'] = np_data.tolist()

                    else:
                        # self.subscribe_data['line'] = numpy.array(data).reshape(line_num, 10, 4)
                        self.subscribe_data[u'line'] = []
                    # print(self.subscribe_data['line'])
                if u"robot" in buf_list:
                    data = []
                    if int(buf_list[buf_list.index(u'robot') + 1]) > 0:
                        for i in xrange(buf_list.index(u'robot') + 2, len(buf_list), 4):
                            data.append(
                                [buf_list[i], buf_list[i + 1], buf_list[i + 2], buf_list[i + 3], buf_list[i + 4]])
                            self.subscribe_data[u'robot'] = data
                            # print(self.subscribe_data['robot'])
                self.get_push_data_ready.set()
