# coding=utf8
from __future__ import absolute_import

import traitlets
from traitlets.config.configurable import Configurable


class Gripper(Configurable):
    open = traitlets.Bool()

    def __init__(self, connection):
        super(Gripper, self).__init__()
        self.connection = connection
        self.level = 1
        self.open = False

    @traitlets.observe(u'open')
    def __observe_status(self, change):
        if self.open:
            self.connection.command(u'robotic_gripper open %d' % (self.level))
        else:
            self.connection.command(u'robotic_gripper close %d' % (self.level))

    def get_gripper_status(self):
        ret = self.connection.command(u'robotic_gripper ?')
        return int(ret[1])
