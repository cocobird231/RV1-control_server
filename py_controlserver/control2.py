import socket
import struct
import time
from subprocess import check_output
from threading import Thread
import random
from Joystick import Joystick
from functools import partial
from concurrent.futures import * 
import binascii
import os
import tkinter as tk
import tkinter.ttk as ttk
import copy
from os.path import expanduser

### coco
import threading
import genMotorPWM

SWITCH_SIGNAL = 0# 0: 5G remote; 1: ros2 remote
ROS2_SIGNAL_INDEX = 0# Switch controller index from ROS2 controller list
ROS2_SIGNAL_SIZE = 0# Size of ROS2 controller list
WIRELESS_BRAKE = False
SAFETY_OVER_CONTROL = False

def JoystickUpCallback(sock):
    global SWITCH_SIGNAL
    SWITCH_SIGNAL = 0
    print('[JoystickUpCallback] Switch to 5G remote control.')
    return

def JoystickDownCallback(sock):
    global SWITCH_SIGNAL
    SWITCH_SIGNAL = 1
    print('[JoystickDownCallback] Switch to ROS2 control.')
    return

def JoystickLeftCallback(sock):
    global ROS2_SIGNAL_INDEX
    if (ROS2_SIGNAL_INDEX > 0) : ROS2_SIGNAL_INDEX -= 1
    print('[JoystickLeftCallback] ROS2_SIGNAL_INDEX: %d' %ROS2_SIGNAL_INDEX)
    return

def JoystickRightCallback(sock):
    global ROS2_SIGNAL_INDEX
    global ROS2_SIGNAL_SIZE
    if (ROS2_SIGNAL_INDEX < ROS2_SIGNAL_SIZE - 1) : ROS2_SIGNAL_INDEX += 1
    print('[JoystickRightCallback] ROS2_SIGNAL_INDEX: %d' %ROS2_SIGNAL_INDEX)
    return

def JoystickBtnACallback(sock):# Enable WIRELESS_BRAKE
    global WIRELESS_BRAKE
    WIRELESS_BRAKE = True
    print('[JoystickBtnACallback] Wireless force brake.')

def JoystickBtnBCallback(sock):# Disable WIRELESS_BRAKE
    global WIRELESS_BRAKE
    WIRELESS_BRAKE = False
    print('[JoystickBtnBCallback] Wireless release brake.')

def JoystickBtnXCallback(sock):# Enable SAFETY_OVER_CONTROL
    SAFETY_OVER_CONTROL = True
    print('[JoystickBtnXCallback] Enable safety over control.')

def JoystickBtnYCallback(sock):# Disable SAFETY_OVER_CONTROL
    SAFETY_OVER_CONTROL = False
    print('[JoystickBtnYCallback] Disable safety over control.')

def JoystickEventsTh(sock):
    joystickProgram = Joystick()
    joystickProgram.init( \
        partial(JoystickUpCallback, sock), \
        partial(JoystickDownCallback, sock), \
        partial(JoystickLeftCallback, sock), \
        partial(JoystickRightCallback, sock), \
        partial(JoystickBtnACallback, sock), \
        partial(JoystickBtnBCallback, sock), \
        partial(JoystickBtnXCallback, sock), \
        partial(JoystickBtnYCallback, sock))
    joystickProgram.run()

__sockSendLock = threading.Lock()
### coco end

serverIp = '192.168.1.42'
TIME_BETWEEN_ALIVE_SIGNALS = 4
TIME_ADDED_BETWEEN_SIGNALS = 0.01

myID= 0

unpackerLength = struct.Struct('I')
int_size = struct.calcsize("I")
executor = ProcessPoolExecutor()

def LoopSendAlive(sock):
    while LoopSendAlive.bWillStop == False:
        data = (0x42, 0x42, 0x42, 0x42)
        SendDataAndGetResponse_(sock, data)
        time.sleep((TIME_BETWEEN_ALIVE_SIGNALS - 2) + random.random()*2)
LoopSendAlive.bWillStop = False

def sockConnect(his_ip):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((his_ip, 10002))
    return sock

def sockConnectImAlive(his_ip):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((his_ip, 10003))
    return sock

def sockConnectCommand(his_ip):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((his_ip, 10004))
    return sock

def SendDataAndGetResponse_(sock, data):
    global __sockSendLock
    packerContent = struct.Struct('I'+ str(len(data)) + 'B')
    packedContent = packerContent.pack(len(data), *data)

    __sockSendLock.acquire()
    try:
        sock.sendall(packedContent)
    except:        
        if data != (0x42, 0x42, 0x42, 0x42):
            print('cannot send "%s"' % binascii.hexlify(packedContent))
            print('cannot send package, shutting down ...')
            #os.system("python3.10 ~/projects/Control-1/RunControl-1.py")
            os._exit(0)
    finally:
        pass
    __sockSendLock.release()

def sendCommandTellId(sock):
    sendOutData = (0x69, 0x74, 0x72, 0x69, 0x00, 0x03, 0x01, 0x00, 0x00, myID)
    SendDataAndGetResponse_(sock, data=sendOutData)

def SendCommandSetAxleAndGetResponse_(sock, deviceId, runDirection = 0, brake = False, pwm = 0):
    print("[SendCommandSetAxleAndGetResponse_] %d %d %d %d" %(deviceId, runDirection, brake, pwm))
    sendOutData = [0x69, 0x74, 0x72, 0x69, 0x00, 0x04, 0x01, 0x00, 0x04]

    #append direction byte
    sendOutData.append(bytes([runDirection])[0])

    if brake:
        sendOutData.append(0x01)
    else:
        sendOutData.append(0x00)

    sendOutData.append(bytes([pwm])[0])

    #apped device id byte
    sendOutData.append(deviceId)
    SendDataAndGetResponse_(sock, data=sendOutData)
