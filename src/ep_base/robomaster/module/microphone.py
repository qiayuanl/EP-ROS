# coding=utf8
from __future__ import absolute_import

import audioop
import sys
import threading
import traitlets
from traitlets.config.configurable import Configurable

from .decoder.audio_decoder import AudioDecoder


class Microphone(Configurable):
    audio = traitlets.Any()
    sample_rate = traitlets.Int()
    DEFAULT_SAMPLE_RATE = 48000

    def __init__(self, connection):
        super(Microphone, self).__init__()
        self.audio = None
        self.sample_rate = 48000
        self.connection = connection
        self.audio_decoder = AudioDecoder()
        self.on = False

    def open(self, sample_rate=48000):
        self.sample_rate = sample_rate
        if self.on:
            print
            u'成功连接到音频模块'
            return
        self.on = True
        self.audio_decoder_thread = threading.Thread(target=self.__audio_decoder_task)
        self.get_audio_ready = threading.Event()
        self.audio_decoder_thread.start()
        self.get_audio_ready.wait()

    # TODO:Bugs to be fixed
    def close(self):
        if self.audio_decoder_thread.is_alive():
            self.on = False
            self.audio_decoder_thread.join()
            print
            u'麦克风连接断开'
        else:
            self.on = False

    def __audio_decoder_task(self):
        package_data = ''
        print
        u'连接音频模块: ',;
        sys.stdout.write(u'')
        self.connection.command(u'audio on')
        if self.connection.start_audio_recv():
            print
            u'成功'
        else:
            self.get_audio_ready.set()
            self.connection.command(u'audio off')
            self.on = False
            print
            u'失败'
            return
        while self.on:
            buff = self.connection.recv_audio_data()
            if buff:
                package_data += buff
                if len(package_data) != 0:
                    pcm_frame = self.audio_decoder.decode(package_data)
                    if pcm_frame:
                        try:
                            if self.sample_rate == self.DEFAULT_SAMPLE_RATE:
                                self.audio = pcm_frame
                            else:
                                self.audio = audioop.ratecv(pcm_frame, 2, 1, 48000, self.sample_rate, None)[0]
                            if not self.get_audio_ready.is_set():
                                self.get_audio_ready.set()
                        except Exception, e:
                            if self.connection.is_shutdown:
                                break
                            print
                            e
                            continue
                    package_data = ''
        self.connection.stop_audio_recv()
        self.connection.command(u'audio off')
        self.get_audio_ready.clear()
