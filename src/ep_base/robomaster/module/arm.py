# coding=utf8
from __future__ import absolute_import

import traitlets
from traitlets.config.configurable import Configurable


class Arm(Configurable):
    x = traitlets.Float(0.1)
    y = traitlets.Float(0.1)

    def __init__(self, connection):
        super(Arm, self).__init__()
        self.connection = connection

    def reset(self):
        self.x = 0.0
        self.y = 0.0

    @traitlets.observe(u'x', u'y')
    def __observe_position(self, change):
        self.connection.command(u'robotic_arm moveto x %f y %f' % (self.x, self.y))

    def stop(self):
        self.connection.command(u'robotic_arm stop')
        ret, now_pos = self.connection.command(u'robotic_arm position ?')
        self.x = float(now_pos.split(u' ')[0])
        self.y = float(now_pos.split(u' ')[1])
