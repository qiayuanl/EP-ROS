# -- coding: utf-8 --
from __future__ import absolute_import

import sys

ros_path = u'/opt/ros/melodic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
import os
from uuid import uuid1
from .decoder.video_decoder import VideoDecoder
from PIL import Image as PImage
import numpy as np
import threading
import traitlets
from traitlets.config.configurable import Configurable


class Camera(Configurable):
    image = traitlets.Any()
    height = traitlets.Int()
    width = traitlets.Int()
    DEFAULT_WIDTH = 1280
    DEFAULT_HEIGHT = 720

    def __init__(self, connection):
        super(Camera, self).__init__()  # initializes traitlets
        self.image = None
        self.height = 720
        self.width = 1280
        self.connection = connection
        self.decoder = VideoDecoder()
        self.on = False

    def open(self, height=360, width=640):
        self.height = height
        self.width = width
        if self.on:
            print u'成功连接到相机模块'
            return
        self.on = True
        self.video_decoder_thread = threading.Thread(target=self.__video_decoder_task)
        self.get_image_ready = threading.Event()
        self.video_decoder_thread.start()
        self.get_image_ready.wait()

    # TODO:Bugs to be fixed
    def close(self):
        if self.video_decoder_thread.is_alive():
            self.on = False
            self.video_decoder_thread.join()
            print u'相机连接断开'
        else:
            self.on = False

    def save_snapshot(self, directory):
        image_path = os.path.join(directory, unicode(uuid1()) + u'.jpg')
        cv2.imwrite(image_path, self.image)
        print u'截图成功，保存至{}'.format(image_path)

    def __video_decoder_task(self):
        package_data = ''
        print u'连接相机模块： ',
        sys.stdout.write(u'')
        self.connection.command(u'stream on')
        if self.connection.start_video_recv():
            print u'成功'
        else:
            self.get_image_ready.set()
            self.connection.command(u'stream off')
            self.on = False
            print u'失败'
            return
        while self.on:
            buff = self.connection.recv_video_data()
            if buff:
                package_data += buff
                if len(buff) != 1460:
                    for frame in self.decoder.decode(package_data):
                        try:
                            img = PImage.fromarray(frame)
                            if self.height == self.DEFAULT_HEIGHT and self.width == self.DEFAULT_WIDTH:
                                self.image = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
                            else:
                                self.image = cv2.resize(cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR),
                                    (self.width, self.height))
                            if not self.get_image_ready.is_set():
                                self.get_image_ready.set()
                        except Exception, e:
                            if self.connection.is_shutdown:
                                break
                            print e
                            continue
                    package_data = ''
        self.connection.stop_video_recv()
        self.connection.command(u'stream off')
        self.get_image_ready.clear()
