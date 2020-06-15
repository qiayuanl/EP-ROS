# coding=utf8
from __future__ import absolute_import

import sys

sys.path.append(u'..')
sys.path.remove(u'/opt/ros/kinetic/lib/python2.7/dist-packages')
import threading

import cv2
import pyaudio

from robomaster import Robot
from robomaster.connection import ConnectionType


def image_cb(change):
    cv2.imshow(u"image", change[u'new'])
    cv2.waitKey(1)


def audio_cb(change):
    global stream
    stream.write(change[u'new'])


u'''
    接口提供了根据不同连接类型`connection_type`连接RoboMaster EP机器人，

- `ConnectionType.USB_DIRECT`: 

    USB直连模式，通过USB线连接EP机器人与电脑

- `ConnectionType.WIFI_DIRECT`： 

    WIFI直连模式，切换EP机器人到直连模式，电脑连接EP机器人的WIFI（例如RMEP-xxxxxx）

- `ConnectionType.WIFI_NETWORKING`：

    WIFI路由模式，切换EP机器人到路由模式，连接EP机器人和PC到同一个网域的路由器上，同时需要手动输入`robot_ip`字段

    '''

robot = Robot(connection_type=ConnectionType.WIFI_NETWORKING, robot_ip=u'192.168.1.102')

u'''
调用`robot`对象中`camera`子对象是摄像头对象

调用`open(height,width)`函数，连接并获取摄像头的图像数据

其中`width`和`height`代表获取图像的宽和高，最大宽x高是1280x720
'''
robot.camera.open(height=360, width=640)

u'''
调用`robot`对象中`microphone`子对象是麦克风对象

调用`open(sample_rate)`函数，连接并获取麦克风的音频数据

其中`sample_rate`代表获取音频的采样率，最大是48000
'''
robot.microphone.open(sample_rate=16000)

u'''撰写如下程序测试音频播放和视频显示程序'''
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16,
                channels=1,
                # Sample rate
                rate=16000,
                # rate=48000,
                output=True)
u'''新图像与音频到来后，会调用observe的回调函数，如image_cb和audio_cb'''
robot.camera.observe(image_cb, u'image')
robot.microphone.observe(audio_cb, u'audio')

event = threading.Event()
try:
    print
    u'Press Ctrl+C to exit'
    event.wait()
except KeyboardInterrupt:
    print
    u'Terminate Visualization and Audio Play'

robot.camera.unobserve(image_cb, u'image')
robot.microphone.unobserve(audio_cb, u'audio')

cv2.destroyAllWindows()
stream.stop_stream()
stream.close()
p.terminate()

robot.close()
