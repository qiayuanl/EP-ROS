# coding=utf8
from __future__ import absolute_import

import sys
import threading
import traitlets
from traitlets.config.configurable import Configurable


class Sound(Configurable):
    subscribe_applause = traitlets.Bool()

    def __init__(self, connection):
        super(Sound, self).__init__()
        self.connection = connection
        self.subscribe_applause = False
        self.recv = 0
        self.data_analysis_thread = threading.Thread(target=self.__sound_event_subscribe_analysis_task)

    @traitlets.observe(u'subscribe_applause')
    def __observe_subscribe_hit(self, change):
        if self.subscribe_applause:
            print
            u'订阅掌声事件： ',;
            sys.stdout.write(u'')
            ret = self.connection.command(u'sound event applause on')
            self.data_analysis_thread.start()
            print
            u'成功'
        else:
            print
            u'取消订阅掌声事件： ',;
            sys.stdout.write(u'')
            self.connection.command(u'sound event applause off')
            if self.data_analysis_thread.is_alive():
                self.on = False
                self.data_analysis_thread.join()
                print
                u'成功'
            else:
                self.on = False

    def __sound_event_subscribe_analysis_task(self):
        self.connection.start_event_recv()

        while self.subscribe_applause:
            buff = self.connection.recv_event_data(u'sound')
            if buff:
                buf_list = buff.split(u' ')
                # print(buf_list)
                self.recv = int(buf_list[3])
                # print('检测到连续%d次拍手' % (self.recv))

    def get_applause(self):
        if self.recv != 0:
            recv = self.recv
            self.recv = 0
            return True, recv
        else:
            return False, 0
