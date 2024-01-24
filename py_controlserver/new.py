import time
import socket
import struct
import random
from enum import Enum

import threading
from threading import Thread
from subprocess import check_output

import rclpy
from rclpy.node import Node

from vehicle_interfaces.msg import Header
from vehicle_interfaces.msg import WheelState
from vehicle_interfaces.msg import ControlInfo
from vehicle_interfaces.msg import Chassis
from vehicle_interfaces.msg import SteeringWheel

from vehicle_interfaces.srv import ControlReg
from vehicle_interfaces.srv import ControlReq
from vehicle_interfaces.srv import ControlSteeringWheel

from vehicle_interfaces.vehicle_interfaces import VehicleServiceNode

import control2
import genMotorPWM
import ros2_utils
import pyIDClient

PACKET_HEADER_SIZE = 22
PACKET_PAYLOAD_SIZE = 1450

class ROS2Controller():
    def __init__(self):
        self.serviceName = ''
        self.node = None
        self.client = None
        self.msg = None

class ControlServer(VehicleServiceNode):# Devinfo, SafetyReq, Timesync
    def __init__(self, params : ros2_utils.ControlParameters):
        super().__init__(params)

        # Store params
        self.__params = params
        self.__exitF = False

        # External IDClient
        self.__externalConnF = False
        self.__remoteF = False
        self.__remoteDevName = ''
        self.__externalTimeout_ms = 5000.0
        self.__externalTimeoutTs = None

        # Internal IDClient
        self.__internalConnF = False
        self.__mIDList = genMotorPWM.motorIDList

        # Stored ROS2 controller info
        self.__ros2ControlInfoDict = {}# { 'service_name' : ControlInfo }
        self.__ros2ControllerDict = {}# { 'serviceName' : ROS2Controller }
        self.__ros2ControllerList = []# ['serviceName']

        # ControlServer service
        self.__regServer = self.create_service(ControlReg, params.serviceName + '_Reg', self.__regServerCallback)
        self.__reqServer = self.create_service(ControlReq, params.serviceName + '_Req', self.__reqServerCallback)

        # Publish selected control signal to topic via WheelState msg
        self.__pub = self.create_publisher(WheelState, params.topicName, 10)
        self.__pubTimer = self.create_timer(params.publishInterval_s, self.__pubTimerCallback)
        self.__pubFrameId = 0

        # Frequently modifying
        self.__currentPubMsg = SteeringWheel()
        self.__externalInputMsg = SteeringWheel()
        self.__ros2InputMsg = SteeringWheel()

        # Output control signal
        self.__outputSignal = None
        self.__externalSignal = None# Chassis()
        self.__ros2Signal = None# Chassis()

        # Threads
        self.__idclientTh = threading.Thread(target=self.__internalIDClientTh)
        self.__idclientTh.start()
        self.__externalTh = threading.Thread(target=self.__externalIDClientTh)
        self.__externalTh.start()
        self.__ros2Th = threading.Thread(target=self.__ros2ControlTh)
        self.__ros2Th.start()
        self.__switchTh = threading.Thread(target=self.__outputSignalSwitch)
        self.__switchTh.start()

    def __del__(self):
        self.__exitF = True
        self.__switchTh.join()
        self.__idclientTh.join()
        self.__externalTh.join()
        self.__ros2Th.join()

    def __allBrake(self):
        for _mid in self.__mIDList:
            control2.SendCommandSetAxleAndGetResponse_(sock=control2.commandSock, deviceId=_mid, runDirection = 2, brake = 1, pwm = 0)

    def __sendSteeringCommand(self, device_id, steering_angle):
        #assert (steering_angle >= -20 and steering_angle <= 20)
        sendOutData = [0x69, 0x74, 0x72, 0x69, 0x00, 0x08, 0x01, 0x00, 0x02]
        distance = int((5 + steering_angle*0.25)*1000)
        distanceLo = (int)(distance%256)
        distanceHi = (int)(distance/256)
        sendOutData.append(distanceHi)
        sendOutData.append(distanceLo)
        sendOutData.append(device_id)
        control2.SendDataAndGetResponse_(sock=control2.commandSock, data=sendOutData)

    def __outputSignalSwitch(self):
        while (not self.__exitF):
            if (not self.__internalConnF):
                time.sleep(self.__params.sendInterval_s)
                continue
            # Wireless controller brake
            if (control2.WIRELESS_BRAKE):
                self.__allBrake()
            else:
                # Decide output signal
                if (control2.SWITCH_SIGNAL == 0):
                    self.__currentPubMsg = self.__externalInputMsg
                    self.__outputSignal = self.__externalSignal
                elif (control2.SWITCH_SIGNAL == 1):
                    self.__currentPubMsg = self.__ros2InputMsg
                    self.__outputSignal = self.__ros2Signal

                # Valid signal
                if (self.__outputSignal):
                    print(self.__outputSignal)

                    safetyPassF = True
                    # Check safety
                    if (control2.SAFETY_OVER_CONTROL):
                        emPs = self.getEmergency("nearest")
                        if (any(emPs)):
                            emForwardF = True if emPs[0] > 0.7 else False
                            emBackwardF = True if emPs[1] > 0.7 else False

                            # check forward or backward TODO: refspeed to pwm
                            dirIndicator = [1 if i > 0 else -1 if i < 0 else 0 for i in self.__outputSignal.drive_motor]
                            if (emForwardF and not (sum(dirIndicator) <= -2)):# Forward
                                safetyPassF = False
                            elif (emBackwardF and not (sum(dirIndicator) >= 2)):# Backward
                                safetyPassF = False

                    if (not safetyPassF):
                        self.__allBrake()
                    else:
                        for _mid, _mcorr, _mval, _sid, _scorr, _sval, _bval, _brk in zip(genMotorPWM.motorIDList, \
                                                                                            genMotorPWM.motorDirectionCorrectionList, \
                                                                                            self.__outputSignal.drive_motor, \
                                                                                            genMotorPWM.steeringIDList, \
                                                                                            genMotorPWM.steeringCorrectionList, \
                                                                                            self.__outputSignal.steering_motor, \
                                                                                            self.__outputSignal.brake_motor, \
                                                                                            self.__outputSignal.parking_signal):
                            _dir = 1 if _mval * _mcorr < 0 else 2
                            _mval = abs(_mval)
                            control2.SendCommandSetAxleAndGetResponse_(sock=control2.commandSock, deviceId=int(_mid), runDirection = int(_dir), brake = int(_brk), pwm = int(_mval))
                            self.__sendSteeringCommand(_sid, _sval * _scorr)
                else:
                    self.__allBrake()
                    print("Not valid signal:", self.__outputSignal)

            time.sleep(self.__params.sendInterval_s)

    def __ros2ControlTh(self):
        brkSignal = Chassis()
        brkSignal.wheel_num = 4
        brkSignal.drive_motor = [0.0, 0.0, 0.0, 0.0]
        brkSignal.steering_motor = [0.0, 0.0, 0.0, 0.0]
        brkSignal.brake_motor = [0.0, 0.0, 0.0, 0.0]
        brkSignal.parking_signal = [True, True, True, True]
        while (not self.__exitF):
            if (control2.SWITCH_SIGNAL == 1):
                if (control2.ROS2_SIGNAL_INDEX < control2.ROS2_SIGNAL_SIZE):
                    sname = self.__ros2ControllerList[control2.ROS2_SIGNAL_INDEX]

                    request = ControlSteeringWheel.Request()
                    request.request = True
                    future = self.__ros2ControllerDict[sname].client.call_async(request)
                    rclpy.spin_until_future_complete(self.__ros2ControllerDict[sname].node, future, timeout_sec=0.02)
                    if (not future.done()):
                        self.__ros2Signal = brkSignal
                        self.get_logger().error('[ControlServer.__ros2ControlTh] Failed to call service: %s.' %sname)
                    else:
                        response = future.result()
                        if (response.response):
                            self.__ros2InputMsg = response.value
                            self.__ros2Signal = genMotorPWM.CalSteeringWheelToChassis(response.value)
                        else:
                            self.__ros2Signal = brkSignal
                else:
                    self.__ros2Signal = brkSignal

            time.sleep(0.05)

    def __externalIDClientTh(self):
        ######## IDClient ########
        prop = pyIDClient.IDServerProp(self.__params.externalIDServerIP, str(self.__params.externalIDServerPort), PACKET_HEADER_SIZE, PACKET_PAYLOAD_SIZE)
        client = pyIDClient.IDClient(prop)# Create IDClient object
        client.setRecvMsgEventHandler(self.__externalIDClientCbFunc, True)# Input function pointer
        self.get_logger().info('[ControlServer.__externalIDClientTh] IDClient object created.')

        brkSignal = Chassis()
        brkSignal.wheel_num = 4
        brkSignal.drive_motor = [0.0, 0.0, 0.0, 0.0]
        brkSignal.steering_motor = [0.0, 0.0, 0.0, 0.0]
        brkSignal.brake_motor = [0.0, 0.0, 0.0, 0.0]
        brkSignal.parking_signal = [True, True, True, True]


        while (not self.__exitF):
            if (not self.__externalConnF):
                while (not client.isServerConn()):# While IDClient not connect to ID server
                    try:
                        client.connToServer()# Create socket connection and receiving thread
                    except:
                        self.get_logger().info('[ControlServer.__externalIDClientTh] Socket connects to %s:%s error. Try again in 1 sec...' %(self.__params.externalIDServerIP, self.__params.externalIDServerPort))
                        time.sleep(1)

                client.regToServer(str(self.__params.externalIDServerDeviceID))# Register to ID server
                self.get_logger().info('[ControlServer.__externalIDClientTh] Register to ID server.')

                client.requestIDTableFromServer()# Request new ID table, no return value
                self.get_logger().info('[ControlServer.__externalIDClientTh] Request ID table from server.')

                time.sleep(0.5)

                table = client.getIDTable()# Get latest ID table
                print("ID table:", table)
                time.sleep(0.5)

                self.get_logger().info('[ControlServer.__externalIDClientTh] Waitting for controller...')
                while (not self.__remoteF):
                    time.sleep(0.1)
                self.get_logger().info('[ControlServer.__externalIDClientTh] Controller connected!')

                self.__externalConnF = True
            else:
                try:
                    if (self.__remoteF and self.__externalTimeoutTs):
                        if ((self.get_clock().now() - self.__externalTimeoutTs).nanoseconds > self.__params.externalTimeout_ms * 1000000):
                            self.get_logger().error('[ControlServer.__externalIDClientTh] External ID client timeout. Reconnect to ID server...')
                            self.__externalConnF = False
                            continue
                        msg = client.getLatestRecvMsg(self.__remoteDevName)
                        splitMsgList = msg.strip().split(':')
                        if (len(splitMsgList) >= 6):# [GEAR, STEERING, THROTTLE, BRAKE, CLUTCH, BUTTON, TIMESTAMP]
                            gear = splitMsgList[0]#             Gear string
                            steering = int(splitMsgList[1])#    Steering wheel value
                            thr = int(splitMsgList[2])#         Throttle pedal value
                            brk = int(splitMsgList[3])#         Brake pedal value
                            clu = int(splitMsgList[4])#         Clutch pedal value
                            button = int(splitMsgList[5])#      Button value

                            getMsg = SteeringWheel()

                            if (gear == 'Park') : getMsg.gear = getMsg.GEAR_PARK
                            elif (gear == 'Reverse') : getMsg.gear = getMsg.GEAR_REVERSE
                            elif (gear == 'Neutral') : getMsg.gear = getMsg.GEAR_NEUTRAL
                            elif (gear == 'Drive') : getMsg.gear = getMsg.GEAR_DRIVE

                            getMsg.steering = steering
                            getMsg.pedal_throttle = thr
                            getMsg.pedal_brake = brk
                            getMsg.pedal_clutch = clu
                            getMsg.func_0 = button
                            self.__externalInputMsg = getMsg

                            self.__externalSignal = genMotorPWM.CalSteeringWheelToChassis(getMsg)

                            if (len(splitMsgList) > 6):
                                retStr = "%d:%d:%d:%d:%d:%d:%d:%d:%d" %(self.__externalSignal.drive_motor[0], \
                                                                        self.__externalSignal.drive_motor[1], \
                                                                        self.__externalSignal.drive_motor[2], \
                                                                        self.__externalSignal.drive_motor[3], \
                                                                        self.__externalSignal.steering_motor[0], \
                                                                        self.__externalSignal.steering_motor[1], \
                                                                        self.__externalSignal.steering_motor[2], \
                                                                        self.__externalSignal.steering_motor[3], \
                                                                        getMsg.func_0)
                                client.sendMsgToClient(self.__remoteDevName, "#" + splitMsgList[-1] + "!" + retStr)# Send timestamp back to remoter
                        else:
                            self.__externalSignal = brkSignal
                    else:
                        self.__externalSignal = brkSignal
                except Exception as e:
                    print('!!!Catch Main Loop Exception!!!', splitMsgList, e)
                    self.__externalSignal = brkSignal
                except:
                    print('!!!Catch Main Loop Exception!!!', splitMsgList)
                    self.__externalSignal = brkSignal
                time.sleep(self.__params.sendInterval_s)

    # This function will be called every client message received
    def __externalIDClientCbFunc(self, idc, deviceName, msg):
        if (msg == 'RemoteRegister' and not self.__remoteF):
            try:
                idc.sendMsgToClient(deviceName, 'ControlRegister')
                self.__remoteDevName = deviceName
                self.__remoteF = True
            except Exception as e:
                print('!!!RecvMsgEventHandler Exception!!!', idc.getIDTable(), e)
                idc.requestIDTableFromServer()
        elif (self.__remoteF):
            self.__externalTimeoutTs = self.get_clock().now()
    
    def __internalIDClientTh(self):
        control2.myID = self.__params.internalIDServerDeviceID
        control2.serverIp = self.__params.internalIDServerIP

        if (not self.__internalConnF):
            connF = False
            while (not connF):
                try:
                    control2.dataSock = control2.sockConnect(self.__params.internalIDServerIP)
                    control2.sendCommandTellId(control2.dataSock)
                    connF = True
                except:
                    self.get_logger().info('[ControlServer.__internalIDClientTh] Retry dataSock connection...')
                    time.sleep(1)

            connF = False
            while (not connF):
                try:
                    control2.imAliveSock = control2.sockConnectImAlive(self.__params.internalIDServerIP)    
                    receivedData = control2.sendCommandTellId(control2.imAliveSock)
                    connF = True
                except:
                    self.get_logger().info('[ControlServer.__internalIDClientTh] Retry imAliveSock connection...')
                    time.sleep(1)

            handler1 = Thread(target=control2.LoopSendAlive, args=(control2.imAliveSock, ))
            handler1.start()

            connF = False
            while (not connF):
                try:
                    control2.commandSock = control2.sockConnectCommand(self.__params.internalIDServerIP)
                    connF = True
                except:
                    self.get_logger().info('[ControlServer.__internalIDClientTh] Retry commandSock connection...')
                    time.sleep(1)

            handler2 = Thread(target=control2.JoystickEventsTh, args=(control2.commandSock, ))
            handler2.start()
            self.__internalConnF = True
        else:
            self.get_logger().info('[ControlServer.__internalIDClientTh] Already connected.')
            time.sleep(1)

    def __regServerCallback(self, request, response):# ControlReg.srv
        self.__ros2ControlInfoDict[request.request.service_name] = request.request

        # Add to ROS2 controller list
        if (request.request.service_name not in self.__ros2ControllerList and request.request.msg_type == ControlInfo.MSG_TYPE_STEERING_WHEEL):
            self.__ros2ControllerDict[request.request.service_name] = ROS2Controller()
            self.__ros2ControllerDict[request.request.service_name].serviceName = request.request.service_name
            self.__ros2ControllerDict[request.request.service_name].node = Node(self.__params.nodeName + '_' + request.request.service_name + '_client')
            self.__ros2ControllerDict[request.request.service_name].client = self.__ros2ControllerDict[request.request.service_name].node.create_client(ControlSteeringWheel, request.request.service_name)

            self.__ros2ControllerList.append(request.request.service_name)
            self.get_logger().info('[ControlServer.__regServerCallback] Add %s to list.' %request.request.service_name)
            control2.ROS2_SIGNAL_SIZE = len(self.__ros2ControllerDict)
        response.response = True
        return response

    def __reqServerCallback(self, request, response):# ControlReq.srv
        response.response = True
        if (request.service_name == 'all'):
            response.control_info_vec = [self.__ros2ControlInfoDict[item] for item in self.__ros2ControlInfoDict]
        elif (request.service_name in self.__ros2ControlInfoDict):
            response.control_info_vec = [self.__ros2ControlInfoDict[request.service_name]]
        else:
            response.response = False
        self.get_logger().info('[ControlServer.__reqServerCallback] Request [%s] controller %s.' % (request.service_name, 'succeeded' if (response.response) else 'failed'))
        return response

    def __pubTimerCallback(self):
        msg = WheelState()
        msg.header.priority = msg.header.PRIORITY_CONTROL
        msg.header.device_type = msg.header.DEVTYPE_STEERINGWHEEL
        msg.header.device_id = self.__params.nodeName
        msg.header.frame_id = self.__pubFrameId
        self.__pubFrameId += 1
        msg.header.stamp_type = self.getTimestampType()
        msg.header.stamp = self.getTimestamp().to_msg()
        msg.header.stamp_offset = self.getCorrectDuration().nanoseconds
        msg.header.ref_publish_time_ms = self.__params.publishInterval_s * 1000.0

        msg.gear = self.__currentPubMsg.gear
        msg.steering = self.__currentPubMsg.steering
        msg.pedal_throttle = self.__currentPubMsg.pedal_throttle
        msg.pedal_brake = self.__currentPubMsg.pedal_brake
        msg.pedal_clutch = self.__currentPubMsg.pedal_clutch
        msg.button = self.__currentPubMsg.func_0
        msg.func = self.__currentPubMsg.func_1

        self.__pub.publish(msg)

def main(params):
    server = ControlServer(params)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(server)
    executorTH = threading.Thread(target=executor.spin, daemon=True)
    executorTH.start()
    executorTH.join()
