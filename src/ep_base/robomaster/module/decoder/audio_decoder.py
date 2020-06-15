from __future__ import absolute_import

from .opus_decoder import opus_decoder


class AudioDecoder(object):
    def __init__(self):
        self.decoder = opus_decoder(frame_size=960, sample_rate=48000, channels=1)

    def decode(self, packet_data):
        return self.decoder.decode(packet_data)


def main():
    decoder = AudioDecoder()
