# coding=utf8
from __future__ import absolute_import

import sys
import threading
import traitlets
from traitlets.config.configurable import Configurable


class Armor(Configurable):
    sensitivity = traitlets.Int()
    subscribe_hit = traitlets.Bool()

    def __init__(self, connection):
        super(Armor, self).__init__()
        self.connection = connection
        self.armor_data = [9, 9]
        self.data_analysis_thread = threading.Thread(target=self.__armor_event_subscribe_analysis_task)

    @traitlets.observe(u'sensitivity')
    def __observe_sensitivity(self, change):
        self.connection.command(u'armor sensitivity %d' % (self.sensitivity))
        print
        u"装甲板灵敏度调整为%d" % (self.sensitivity)

    @traitlets.observe(u'subscribe_hit')
    def __observe_subscribe_hit(self, change):
        if self.subscribe_hit:
            print
            u'订阅装甲板事件： ',;
            sys.stdout.write(u'')
            ret = self.connection.command(u'armor event hit on')
            self.data_analysis_thread.start()
            print
            u'成功'
        else:
            print
            u'取消订阅装甲板事件： ',;
            sys.stdout.write(u'')
            self.connection.command(u'armor event hit off')
            if self.data_analysis_thread.is_alive():
                self.on = False
                self.data_analysis_thread.join()
                print
                u'成功'
            else:
                self.on = False

    def __armor_event_subscribe_analysis_task(self):
        self.connection.start_event_recv()

        while self.subscribe_hit:
            buff = self.connection.recv_event_data(u'armor')
            if buff:
                buf_list = buff.split(u' ')
                # print(buf_list)
                self.armor_data = [int(buf_list[3]), int(buf_list[4])]
                # print('%d号装甲受到敲击,敲击方式为第%d种' % (self.armor_data[0], self.armor_data[1]))

    def get_hit(self):
        if self.armor_data != [9, 9]:
            recv = self.armor_data.copy()
            self.armor_data = [0, 0]
            return True, recv
        else:
            return False, 0
