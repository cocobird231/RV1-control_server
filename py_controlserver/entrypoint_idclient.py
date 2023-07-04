import rclpy
from ros2_utils import ControlParameters, WheelStatePublisher
import idclient_utils
from idclient_utils import *
import control
import genMotorPWM

import socket
import struct
import time
from subprocess import check_output
from threading import Thread
import random

from vehicle_interfaces import safety

__SafetyGndEmP = 0

def GetGndDetectEmPTh(safetyNode, gndDetectNodeName, sock):
    global __SafetyGndEmP
    mIDList = genMotorPWM.motorIDList
    while(True):
        __SafetyGndEmP = safetyNode.getEmergency(gndDetectNodeName)
        if (__SafetyGndEmP > 0.7):
            control.__safetyOverControl = False
            if (control.__safetyPermission):
                print('[GetGndDetectEmPTh] Safety Over Control. emP: %f' %__SafetyGndEmP)
                for _m_id in mIDList:
                    control.SendCommandSetAxleAndGetResponse_(sock=sock, deviceId=_m_id, runDirection = 2, brake = 1, pwm = 0)
        else:
            control.__safetyOverControl = True
        time.sleep(0.3)# 3Hz


def main(params):
    ######## IDClient ########
    prop = pyIDClient.IDServerProp(params.externalIDServerIP, str(params.externalIDServerPort), PACKET_HEADER_SIZE, PACKET_PAYLOAD_SIZE)
    client = pyIDClient.IDClient(prop)# Create IDClient object
    print("IDClient object created.")

    while (not client.isServerConn()):# While IDClient not connect to ID server
        try:
            client.connToServer()# Create socket connection and receiving thread
        except:
            print("Socket connects to %s:%s error. Try again in 1 sec..." %(params.externalIDServerIP, str(params.externalIDServerPort)))
            time.sleep(1)

    client.setRecvMsgEventHandler(idclient_utils.func, True)# Input function pointer
    client.regToServer(str(params.externalIDServerDeviceID))# Register to ID server
    print("Register to ID server.")

    client.requestIDTableFromServer()# Request new ID table, no return value
    print("Request ID table from server.")
    time.sleep(0.5)

    table = client.getIDTable()# Get latest ID table
    print("ID table:", table)
    time.sleep(0.5)
    print("Waitting for controller...")
    while (not idclient_utils.isRemoteF):
        time.sleep(0.1)
    print("Controller connected!")

    ######## ROS2 ########
    wsPub = WheelStatePublisher(params)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(wsPub)
    executorTH = threading.Thread(target=executor.spin, daemon=True)
    executorTH.start()

    safety_node = safety.SafetyNode(params.nodeName, params.safetyService)
    exec_safety = rclpy.executors.MultiThreadedExecutor()
    exec_safety.add_node(safety_node)
    exec_safety_th = threading.Thread(target=exec_safety.spin, daemon=True)
    exec_safety_th.start()

    ######## Control ########
    control.myID = params.internalIDServerDeviceID
    control.serverIp = params.internalIDServerIP
    connF = False
    while (not connF):
        try:
            control.dataSock = control.sockConnect(params.internalIDServerIP)
            control.sendCommandTellId(control.dataSock)
            connF = True
        except:
            print('Retry dataSock connection...')
            time.sleep(1)

    connF = False
    while (not connF):
        try:
            control.imAliveSock = control.sockConnectImAlive(params.internalIDServerIP)    
            receivedData = control.sendCommandTellId(control.imAliveSock)
            connF = True
        except:
            print('Retry imAliveSock connection...')
            time.sleep(1)

    handler1 = Thread(target=control.LoopSendAlive, args=(control.imAliveSock, ))
    handler1.start()

    connF = False
    while (not connF):
        try:
            control.commandSock = control.sockConnectCommand(params.internalIDServerIP)
            connF = True
        except:
            print('Retry commandSock connection...')
            time.sleep(1)
    
    handler2 = Thread(target=control.JoystickEventsTh, args=(control.commandSock, ))
    handler2.start()
    
    # Safety
    getGndDetectEmPTh = Thread(target=GetGndDetectEmPTh, args=(safety_node, params.gndDetectNode, control.commandSock))
    getGndDetectEmPTh.start()


    ######## Main Loop ########
    while (True):
        try:
            if (idclient_utils.isRemoteF):
                msg = client.getLatestRecvMsg(idclient_utils.remoteDevName)
                splitMsgList = msg.strip().split(':')
                if (len(splitMsgList) >= 6):# [GEAR, STEERING, THROTTLE, BRAKE, CLUTCH, BUTTON, TIMESTAMP]
                    gear = splitMsgList[0]#             Gear string
                    steering = int(splitMsgList[1])#    Steering wheel value
                    thr = int(splitMsgList[2])#        Throttle pedal value
                    brk = int(splitMsgList[3])#        Brake pedal value
                    clu = int(splitMsgList[4])#        Clutch pedal value
                    button = int(splitMsgList[5])#      Button value

                    wsPub.setMsg(gear, steering, thr, brk, clu, button, 0)# Pass data to ros2 publisher
                    motorValList, steeringValList, steeringType = genMotorPWM.ConvertSteeringWheelToCommand(gear, steering, thr, brk, clu, button)
                    
                    if (len(splitMsgList) > 6):
                        retStr = "%d:%d:%d:%d:%d:%d:%d:%d:%d" %(motorValList[0], motorValList[1], motorValList[2], motorValList[3], \
                            steeringValList[0], steeringValList[1], steeringValList[2], steeringValList[3], steeringType)
                        client.sendMsgToClient(idclient_utils.remoteDevName, "#" + splitMsgList[-1] + "!" + retStr)# Send timestamp back to remoter
                else:
                    genMotorPWM.ConvertSteeringWheelToCommand('Park', 0, 0, 0, 0, 1)
            else:
                genMotorPWM.ConvertSteeringWheelToCommand('Park', 0, 0, 0, 0, 1)
        except:
            print('!!!Catch Main Loop Exception!!!', splitMsgList)
            genMotorPWM.ConvertSteeringWheelToCommand('Park', 0, 0, 0, 0, 1)
        time.sleep(params.sendInterval)
    genMotorPWM.ConvertSteeringWheelToCommand('Park', 0, 0, 0, 0, 1)
    handler1.join()
    executorTH.join()
