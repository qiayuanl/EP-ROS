# coding=utf8
from __future__ import absolute_import

import Queue
import re
import select
import socket
import sys
import threading
import time
from enum import Enum
from itertools import ifilter


class ConnectionType(Enum):
    WIFI_DIRECT = 1
    WIFI_NETWORKING = 2
    USB_DIRECT = 3


class Connection(object):
    u"""
    Create a Connection object with a given robot ip.
    """
    VIDEO_PORT = 40921
    AUDIO_PORT = 40922
    CTRL_PORT = 40923
    PUSH_PORT = 40924
    EVENT_PORT = 40925
    IP_PORT = 40926

    WIFI_DIRECT_IP = u'192.168.2.1'
    WIFI_NETWORKING_IP = u''
    USB_DIRECT_IP = u'192.168.42.2'

    def __init__(self, connection_type, robot_ip=u''):

        self.connection_type = connection_type
        self.robot_ip = robot_ip
        self.video_socket = None
        self.audio_socket = None
        self.ctrl_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ctrl_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.event_socket = None
        self.push_socket = None
        self.ip_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.cmd_socket_list = [self.ctrl_socket]
        self.msg_queue = {
            u'audio': Queue.Queue(32),
            u'video': Queue.Queue(32),
            u'chassis': Queue.Queue(32),
            u'gimbal': Queue.Queue(32),
            u'AI': Queue.Queue(32),
            u'armor': Queue.Queue(32),
            u'sound': Queue.Queue(32)

        }
        self.ack_dict = {}
        self.seq = 0
        self.cmd_socket_recv_thread = threading.Thread(target=self.__socket_recv_task)
        self.is_shutdown = True

    def __get_robot_ip(self, timeout=None):
        u"""
        Get the robot ip from ip broadcat port
        If optional arg 'timeout' is None (the default), block if necessary until
        get robot ip from broadcast port. If 'timeout' is a non-negative number,
        it blocks at most 'timeout' seconds and reuturn None if no data back from
        robot broadcast port within the time. Otherwise, return the robot ip
        immediately.
        """
        self.ip_socket.settimeout(timeout)
        msg = None
        try:
            msg, addr = self.ip_socket.recvfrom(1024)
        except Exception, e:
            print
            u'Get robot ip failed, please check the robot networking-mode and connection !'
        else:
            msg = msg.decode(u'utf-8')
            msg = msg[msg.find(u'robot ip ') + len(u'robot ip '):]

        return msg

    def open(self):
        u"""
        Open the connection
        It will connect the control port and event port with TCP and start a data
        recvive thraed.
        """
        if self.is_shutdown:
            if self.connection_type is ConnectionType.WIFI_DIRECT:
                self.robot_ip = Connection.WIFI_DIRECT_IP
            elif self.connection_type is ConnectionType.USB_DIRECT:
                self.robot_ip = Connection.USB_DIRECT_IP
            elif self.connection_type is ConnectionType.WIFI_NETWORKING:
                if self.robot_ip == u'':
                    robot_ip = self.__get_robot_ip(timeout=10)
                    if robot_ip:
                        self.robot_ip = robot_ip
                    else:
                        descrip = u'Can not find available ip address in WIFI_NETWORKING mode, please check connection again.'
                        return False, descrip

            self.ctrl_socket.settimeout(2)
            try:
                self.ctrl_socket.connect((self.robot_ip, Connection.CTRL_PORT))
                # self.event_socket.connect((self.robot_ip, Connection.EVENT_PORT))
                self.device_ip = self.ctrl_socket.getsockname()[0]
                # self.push_socket.bind((self.device_ip, Connection.PUSH_PORT))
            #                 self.ip_socket.bind((self.device_ip, Connection.IP_PORT))
            except Exception, e:
                descrip = u'Connection failed in connect, the reason is %s' % e
                return False, descrip
            else:
                self.is_shutdown = False
                self.cmd_socket_recv_thread.start()
                descrip = u'Connection successful'
                return True, descrip
        else:
            descrip = u'Connection successful'
            return True, descrip

    def close_socket(self, socket_obj):
        socket_obj.shutdown(socket.SHUT_RDWR)
        socket_obj.close()

    def close(self):
        u"""
        Close the connection
        """
        if self.cmd_socket_recv_thread.is_alive():
            self.is_shutdown = True
            self.cmd_socket_recv_thread.join()
            print
            u'机器人连接断开'
        else:
            self.is_shutdown = True

    def start_video_recv(self):
        if self.is_shutdown:
            return False
        self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if self.video_socket not in self.cmd_socket_list:
            self.video_socket.settimeout(2)
            try:
                self.video_socket.connect((self.robot_ip, Connection.VIDEO_PORT))
            except Exception, e:
                print
                u'Connection failed in video, the reason is %s' % e
                return False
            self.cmd_socket_list.append(self.video_socket)
        return True

    def stop_video_recv(self):
        if self.video_socket in self.cmd_socket_list:
            self.cmd_socket_list.remove(self.video_socket)
        time.sleep(0.1)
        self.close_socket(self.video_socket)
        return True

    def start_audio_recv(self):
        if self.is_shutdown:
            return False
        self.audio_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if self.audio_socket not in self.cmd_socket_list:
            self.audio_socket.settimeout(5)
            try:
                self.audio_socket.connect((self.robot_ip, Connection.AUDIO_PORT))
            except Exception, e:
                print
                u'Connection failed in audio, the reason is %s' % e
                return False
            self.cmd_socket_list.append(self.audio_socket)
        return True

    def stop_audio_recv(self):
        if self.audio_socket in self.cmd_socket_list:
            self.cmd_socket_list.remove(self.audio_socket)
        time.sleep(0.1)
        self.close_socket(self.audio_socket)
        return True

    def start_push_recv(self):
        if self.is_shutdown:
            return False
        if not self.push_socket:
            self.push_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.push_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if self.push_socket not in self.cmd_socket_list:
            self.push_socket.settimeout(5)
            try:
                self.push_socket.bind((self.device_ip, Connection.PUSH_PORT))
            except Exception, e:
                print
                u'open failed in push, the reason is %s' % e
                return False
            self.cmd_socket_list.append(self.push_socket)
        return True

    def stop_push_recv(self):
        if self.push_socket in self.cmd_socket_list:
            self.cmd_socket_list.remove(self.push_socket)
        time.sleep(0.1)
        self.close_socket(self.push_socket)
        return True

    def start_event_recv(self):
        if self.is_shutdown:
            return False
        self.event_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.event_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        if self.event_socket not in self.cmd_socket_list:
            self.event_socket.settimeout(5)
            try:
                self.event_socket.connect((self.robot_ip, Connection.EVENT_PORT))
            except Exception, e:
                print
                u'subscribe failed in event, the reason is %s' % e
                return False
            self.cmd_socket_list.append(self.event_socket)
        return True

    def stop_event_recv(self):
        if self.event_socket in self.cmd_socket_list:
            self.cmd_socket_list.remove(self.event_socket)
        time.sleep(0.1)
        self.close_socket(self.event_socket)
        return True

    def command_on(self):
        print
        u'启动SDK： ',;
        sys.stdout.write(u'')
        seq, ret = self.__send_data(self.ctrl_socket, u'command')
        print
        u'成功'
        return True

    def command(self, msg):
        ctrl_command = msg.find(u'?') == -1
        seq, ret = self.__send_data(self.ctrl_socket, msg)
        ack = self.__wait_for_ack(seq)
        if ack is None:
            print
            u'.',;
            sys.stdout.write(u'')
            return self.command(msg)
        elif ctrl_command:
            if ack == u'ok':
                return True, ack
            else:
                return False, ack
        else:
            return True, ack

    def recv_video_data(self, timeout=None, latest_data=False):
        u"""
        Receive control data
        If optional arg 'timeout' is None (the default), block if necessary until
        get data from control port. If 'timeout' is a non-negative number,
        it blocks at most 'timeout' seconds and reuturn None if no data back from
        robot video port within the time. Otherwise, return the data immediately.

        If optional arg 'latest_data' is set to True, it will return the latest
        data, instead of the data in queue tail.
        """
        return self.__recv_data(u'video', timeout, latest_data)

    def recv_audio_data(self, timeout=None, latest_data=False):
        u"""
        Receive control data
        If optional arg 'timeout' is None (the default), block if necessary until
        get data from control port. If 'timeout' is a non-negative number,
        it blocks at most 'timeout' seconds and reuturn None if no data back from
        robot video port within the time. Otherwise, return the data immediately.

        If optional arg 'latest_data' is set to True, it will return the latest
        data, instead of the data in queue tail.
        """
        return self.__recv_data(u'audio', timeout, latest_data)

    def recv_push_data(self, moudle, timeout=None, latest_data=False):
        u"""
        Receive push data

        If optional arg 'timeout' is None (the default), block if necessary until
        get data from push port. If 'timeout' is a non-negative number,
        it blocks at most 'timeout' seconds and reuturn None if no data back from
        robot push port within the time. Otherwise, return the data immediately.

        If optional arg 'latest_data' is set to True, it will return the latest
        data, instead of the data in queue tail.
        """
        return self.__recv_data(moudle, timeout, latest_data)

    def recv_event_data(self, moudle, timeout=None, latest_data=False):
        u"""
        Receive event data

        If optional arg 'timeout' is None (the default), block if necessary until
        get data from push port. If 'timeout' is a non-negative number,
        it blocks at most 'timeout' seconds and reuturn None if no data back from
        robot push port within the time. Otherwise, return the data immediately.

        If optional arg 'latest_data' is set to True, it will return the latest
        data, instead of the data in queue tail.
        """
        return self.__recv_data(moudle, timeout, latest_data)

    def __send_data(self, socket_obj, data):
        assert not self.is_shutdown, u'CONECTION INVALID'
        self.seq += 1
        seq = self.seq
        data = data + u' seq %d;' % seq
        return seq, socket_obj.send(data.encode(u'utf-8'))

    def __wait_for_ack(self, seq, timeout=2):
        while self.ack_dict.get(seq) is None and timeout >= 0:
            time.sleep(0.01)
            timeout -= 0.01
        if self.ack_dict.get(seq) is None:
            return None
        else:
            return self.ack_dict.pop(seq)

    def __recv_data(self, module, timeout, latest_data):
        if self.is_shutdown:
            return None
        msg = None
        if latest_data:
            while self.msg_queue[module].qsize() > 1:
                self.msg_queue[module].get()
        try:
            msg = self.msg_queue[module].get(timeout=timeout)
        except Exception, e:
            return None
        else:
            return msg

    def __socket_recv_task(self):
        while not self.is_shutdown:

            rlist, _, _ = select.select(self.cmd_socket_list, [], [], 2)

            for s in rlist:

                msg, addr = s.recvfrom(4096)
                if s is self.video_socket:
                    if self.msg_queue[u'video'].full():
                        self.msg_queue[u'video'].get()
                    self.msg_queue[u'video'].put(msg)
                elif s is self.audio_socket:
                    if self.msg_queue[u'audio'].full():
                        self.msg_queue[u'audio'].get()
                    self.msg_queue[u'audio'].put(msg)
                elif s is self.ctrl_socket:
                    msg = msg.decode(u'utf-8')
                    self.__unpack_protocol(msg)
                elif s is self.push_socket:
                    msg = msg.decode(u'utf-8')
                    msg_list = msg.split(u' ')
                    if u"gimbal" in msg_list:
                        if self.msg_queue[u'gimbal'].full():
                            self.msg_queue[u'gimbal'].get()
                        self.msg_queue[u'gimbal'].put(msg)
                    elif u"chassis" in msg_list:
                        if self.msg_queue[u'chassis'].full():
                            self.msg_queue[u'chassis'].get()
                        self.msg_queue[u'chassis'].put(msg)
                    elif u"AI" in msg_list:
                        if self.msg_queue[u'AI'].full():
                            self.msg_queue[u'AI'].get()
                        self.msg_queue[u'AI'].put(msg)
                elif s is self.event_socket:
                    msg = msg.decode(u'utf-8')
                    msg_list = msg.split(u' ')
                    if u"armor" in msg_list:
                        if self.msg_queue[u'armor'].full():
                            self.msg_queue[u'armor'].get()
                        self.msg_queue[u'armor'].put(msg)
                    elif u"sound" in msg_list:
                        if self.msg_queue[u'sound'].full():
                            self.msg_queue[u'sound'].get()
                        self.msg_queue[u'sound'].put(msg)

        for s in self.cmd_socket_list:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except Exception, e:
                pass

    def __unpack_protocol(self, msg):
        msg_list = list(ifilter(None, re.split(ur'( seq \d+)', msg)))
        if len(msg_list) == 2:
            info = msg_list[0]
            seq = int(msg_list[1].split(u' ')[2])
            self.ack_dict[seq] = info
