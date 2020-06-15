from __future__ import absolute_import
from __future__ import division

import cv2
import numpy as np
import signal
import sys
from robomaster import Robot
from robomaster.connection import ConnectionType
from robomaster.module import RobotModeType


def process(image):
    # RGB转HSV色彩空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #########识别黄色###############
    # low_hsv = np.array([26,43,46])
    # high_hsv = np.array([34,255,255])

    #########识别红色###############
    # low_hsv = np.array([0,43,46])
    # high_hsv = np.array([5,255,255])

    #########识别绿色###############
    # low_hsv = np.array([35,43,46])
    # high_hsv = np.array([77,255,255])

    #########ipad识别绿色###############
    low_hsv = np.array([46, 218, 116])
    high_hsv = np.array([85, 255, 255])

    # # 获取trackbar位置
    low_hsv[0] = cv2.getTrackbarPos(u'lh', u'Trackbar')
    low_hsv[1] = cv2.getTrackbarPos(u'ls', u'Trackbar')
    low_hsv[2] = cv2.getTrackbarPos(u'lv', u'Trackbar')
    high_hsv[0] = cv2.getTrackbarPos(u'hh', u'Trackbar')
    high_hsv[1] = cv2.getTrackbarPos(u'hs', u'Trackbar')
    high_hsv[2] = cv2.getTrackbarPos(u'hv', u'Trackbar')
    #
    # p5=cv2.getTrackbarPos('p5', 'Trackbar')
    # p7=cv2.getTrackbarPos('p7', 'Trackbar')
    # min=cv2.getTrackbarPos('min', 'Trackbar')
    # max=cv2.getTrackbarPos('max', 'Trackbar')

    mask = cv2.inRange(hsv, lowerb=low_hsv, upperb=high_hsv)
    cv2.namedWindow(u'shibie_image', cv2.WINDOW_AUTOSIZE)
    cv2.imshow(u'shibie_image', mask)
    dst = cv2.blur(mask, (1, 16))  # 均值模糊 : 去掉提取完颜色的随机噪声图片

    circles = cv2.HoughCircles(dst, cv2.HOUGH_GRADIENT, 1, 50, param1=100, param2=24, minRadius=20, maxRadius=50)
    # circles = cv2.HoughCircles(dst, cv2.HOUGH_GRADIENT, 1, p5, param1=100, param2=p7, minRadius=min, maxRadius=max)

    pos = [0, 0]
    if circles is not None:
        for i in circles[0, :]:
            cv2.circle(image, (i[0], i[1]), i[2], (255, 255, 255), 2)  # 画圆
            cv2.circle(image, (i[0], i[1]), 2, (0, 0, 0), 2)  # 画圆心

            pos[0] = i[0]
            pos[1] = i[1]
            # print('x='+str(pos[0]),end='\t')
            # print('y='+str(pos[1]))
            ret = True
    else:
        print
        u'未识别到圆形图案'
        ret = False
    return image, pos, ret


def out_limit(value, maxout, minout):
    if (value > maxout):
        value = maxout
    elif (value < minout):
        value = minout
    return value


def test():
    robot = Robot(connection_type=ConnectionType.USB_DIRECT)
    robot.camera.open()
    robot.status.mode = RobotModeType.FREE

    def exit(signum, frame):
        robot.close()

    signal.signal(signal.SIGINT, exit)
    signal.signal(signal.SIGTERM, exit)

    def nothing(x):
        pass

    # 创建滑动条
    cv2.namedWindow(u'Trackbar')
    cv2.resizeWindow(u'Trackbar', 500, 1000)

    cv2.createTrackbar(u'lh', u'Trackbar', 30, 180, nothing)
    cv2.createTrackbar(u'ls', u'Trackbar', 218, 255, nothing)
    cv2.createTrackbar(u'lv', u'Trackbar', 116, 255, nothing)
    cv2.createTrackbar(u'hh', u'Trackbar', 68, 180, nothing)
    cv2.createTrackbar(u'hs', u'Trackbar', 255, 255, nothing)
    cv2.createTrackbar(u'hv', u'Trackbar', 255, 255, nothing)

    cv2.createTrackbar(u'p5', u'Trackbar', 50, 100, nothing)
    cv2.createTrackbar(u'p7', u'Trackbar', 25, 50, nothing)
    # #注：24时会检测到方形，26时会经常检测不到，如果用更大的图片效果会好
    cv2.createTrackbar(u'min', u'Trackbar', 10, 200, nothing)
    cv2.createTrackbar(u'max', u'Trackbar', 50, 200, nothing)

    while True:
        frame = robot.camera.image
        if frame.any():
            result, pos, p_ret = process(frame)
            cv2.imshow(u"result", result)

            cv2.waitKey(1)

            # 720*1280
            if p_ret is True:
                x_pos = 1280 / 2 / 2 - pos[0]
                y_pos = 720 / 2 / 2 - pos[1]
                print
                u'x=' + unicode(x_pos),;
                sys.stdout.write(u'\t')
                print
                u'y=' + unicode(y_pos)
                Kp = 0.8
                xout = Kp * x_pos
                yout = Kp * y_pos
                xout = out_limit(xout, 50, -50)
                yout = out_limit(yout, 50, -50)
                robot.gimbal.speed.pitch = yout
                robot.gimbal.speed.yaw = -xout

            else:
                robot.gimbal.speed.pitch = 0
                robot.gimbal.speed.yaw = 0
        else:
            print
            u'未获取到图像信息'
            robot.command(u'gimbal speed p 0 y 0;')


if __name__ == u'__main__':
    test()
