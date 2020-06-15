# coding=utf8
from __future__ import absolute_import

import traitlets
from enum import Enum
from traitlets.config.configurable import Configurable


class RobotModeType(Enum):
    CHASSIS_LEAD = 1
    GIMBAL_LEAD = 2
    FREE = 3


class Status(Configurable):
    mode = traitlets.Enum(RobotModeType)

    def __init__(self, connection):
        super(Status, self).__init__()
        self.connection = connection

    @traitlets.observe(u'mode')
    def __observe_mode(self, change):
        if self.mode == RobotModeType.CHASSIS_LEAD:
            self.connection.command(u'robot mode chassis_lead;')
        elif self.mode == RobotModeType.GIMBAL_LEAD:
            self.connection.command(u'robot mode gimbal_lead;')
        elif self.mode == RobotModeType.FREE:
            self.connection.command(u'robot mode free;')

    def get_battery_power(self):
        ret = self.connection.command(u'robot battery ?')
        return ret[1]
