# coding=utf8
from __future__ import absolute_import

import traitlets
from traitlets.config.configurable import Configurable


class Blaster(Configurable):
    fire = traitlets.Bool()

    def __init__(self, connection):
        super(Blaster, self).__init__()  # initializes traitlets
        self.connection = connection
        self.fire = False

    @traitlets.observe(u'fire')
    def __observe_speed(self, change):
        if change[u'new']:
            self.connection.command(u'blaster fire')
            self.fire = False
