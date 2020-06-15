from __future__ import absolute_import

import traitlets
from traitlets.config.configurable import Configurable


class LED(Configurable):
    color = traitlets.Tuple()

    def __init__(self, connection):
        super(LED, self).__init__()
        self.connection = connection

    @traitlets.observe(u'color')
    def __observe_color(self, change):
        self.set_color(change[u'new'])

    def set_color(self, color=(255, 0, 0)):
        self.connection.command(u'led control comp top_all|bottom_all r %d g %d b %d effect solid'
                                % (color[0], color[1], color[2]))

    def get_color(self):
        return self.color
