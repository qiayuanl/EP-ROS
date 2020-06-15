from __future__ import absolute_import
from __future__ import division

import numpy as np

from .libh264decoder import H264Decoder, disable_logging


class VideoDecoder(object):
    def __init__(self):
        self.decoder = H264Decoder()
        disable_logging()

    def decode(self, packet_data):
        res_frame_list = []
        frames = self.decoder.decode(packet_data)
        for framedata in frames:
            (frame, w, h, ls) = framedata
            if frame is not None:
                frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep=u'')
                frame = (frame.reshape((h, int(ls / 3), 3)))
                frame = frame[:, :w, :]
                res_frame_list.append(frame)
        return res_frame_list


def main():
    decoder = VideoDecoder()
