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
#import numpy as np
from pathlib import Path
from os.path import expanduser


### coco
import threading
import genMotorPWM
__controlPermission = True
__safetyPermission = True
__safetyOverControl = True

def JoystickUpCallback(sock):
    print('[JoystickUpCallback]')
    return

def JoystickDownCallback(sock):
    print('[JoystickDownCallback]')
    return

def JoystickLeftCallback(sock):
    print('[JoystickLeftCallback]')
    return

def JoystickRightCallback(sock):
    print('[JoystickRightCallback]')
    return

def JoystickBtnACallback(sock):# Lock contorl permission and Park
    global __controlPermission
    mIDList = genMotorPWM.motorIDList
    print('[JoystickBtnACallback] Lock Permission')
    __controlPermission = False
    for _m_id in mIDList:
        SendCommandSetAxleAndGetResponse_(sock=sock, deviceId=_m_id, runDirection = 2, brake = 1, pwm = 0)
    return

def JoystickBtnBCallback(sock):# Release contorl permission
    global __controlPermission
    print('[JoystickBtnBCallback] Release Permission')
    __controlPermission = True
    return

def JoystickBtnXCallback(sock):# Lock safety permission
    global __safetyPermission
    print('[JoystickBtnXCallback] Disable Safety Over Control')
    __safetyPermission = False
    return

def JoystickBtnYCallback(sock):# Release safety permission
    global __safetyPermission
    print('[JoystickBtnYCallback] Enable Safety Over Control')
    __safetyPermission = True
    return

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


#myID = 0x01 #Parking馬達
#myID = 0x0B #輪軸馬達
#myID = 0x15 #UPS

home = expanduser('~') + "/tmp/"
Path(home).mkdir(parents=True, exist_ok=True)

unpackerLength = struct.Struct('I')
int_size = struct.calcsize("I")
executor = ProcessPoolExecutor()

def LoopSendAlive(sock):
    while LoopSendAlive.bWillStop == False:
        #print('send Alive socket')
        data = (0x42, 0x42, 0x42, 0x42)
        #receuvedData = SendDataAndGetResponse(sock, data)
        SendDataAndGetResponse(sock, data)
        #print('get ALIVE response ', receuvedData)
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
    
#def SendDataAndGetResponse(sock, data):
#    handler = Thread(target=SendDataAndGetResponse_, args=(sock, data))
#    handler.start()

def SendDataAndGetResponse_(sock, data):
    global __sockSendLock
    packerContent = struct.Struct('I'+ str(len(data)) + 'B')
    packedContent = packerContent.pack(len(data), *data)

    __sockSendLock.acquire()
    try:
        # Send data
        #if data != (0x42, 0x42, 0x42, 0x42):
        #    print('going to send out "%s"' % binascii.hexlify(packedContent))
        
        sock.sendall(packedContent)
        
        #if data != (0x42, 0x42, 0x42, 0x42):
        #    print('sent "%s"' % binascii.hexlify(packedContent))

        # Receive data
        '''
        data1 = sock.recv(unpackerLength.size)
        #print('received "%s"' % binascii.hexlify(data1))
        unpacked_length = unpackerLength.unpack(data1)[0]
        data2 = sock.recv(unpacked_length)
        unpackerContent = struct.Struct(str(unpacked_length) + 'B')
        unpacked_content = unpackerContent.unpack(data2)
        #print('unpacked content:', unpacked_content)  
        '''
        
    except:        
        if data != (0x42, 0x42, 0x42, 0x42):
            print('cannot send "%s"' % binascii.hexlify(packedContent))
            print('cannot send package, shutting down ...')
            #os.system("python3.10 ~/projects/Control-1/RunControl-1.py")
            os._exit(0)

                
        
        
    finally:
        #print('closing socket')
        #sock.close()
        pass
    
    __sockSendLock.release()
    #return unpacked_content

def sendCommandTellId(sock):
    sendOutData = (0x69, 0x74, 0x72, 0x69, 0x00, 0x03, 0x01, 0x00, 0x00, myID)
    #receivedData = SendDataAndGetResponse(sock, data=sendOutData)
    SendDataAndGetResponse(sock, data=sendOutData)
    #return receivedData

def HiLevelCommandCurrent():
    print('送出指令')
    labelBulletin.config(text = '送出指令')
    currentSettings = LoadCommandFromFile(home + '/current_settings')
    for i in range(0, 4):
        print('send out: direction = ', currentSettings[i]['direction'], ', pwm = ', currentSettings[i]['pwm'], ', brake = ', currentSettings[i]['brake'])
        SendCommandSetAxleAndGetResponse(sock=commandSock, deviceId=11+i, runDirection = currentSettings[i]['direction'], brake =  currentSettings[i]['brake'], pwm = currentSettings[i]['pwm'])
        
def HiLevelCommandBrakeAll():
    print('停止')
    labelBulletin.config(text = '停止')
    for i in range(0, 4):
        SendCommandSetAxleAndGetResponse(sock=commandSock, deviceId=11+i, runDirection = 0, brake = 1, pwm = 0)    

def HiLevelCommandGoForward(sock):
    print('前進')
    labelBulletin.config(text = '前進')
    currentSettings = LoadCommandFromFile(home + '/forward_settings')
    for i in range(0, 4):
            SendCommandSetAxleAndGetResponse(sock=sock, deviceId=11+i, runDirection = currentSettings[i]['direction'], brake =  currentSettings[i]['brake'], pwm = currentSettings[i]['pwm'])

def HiLevelCommandGoBackward(sock):
    print('後退')
    labelBulletin.config(text = '後退')
    currentSettings = LoadCommandFromFile(home + '/backward_settings')
    for i in range(0, 4):
            SendCommandSetAxleAndGetResponse(sock=sock, deviceId=11+i, runDirection = currentSettings[i]['direction'], brake =  currentSettings[i]['brake'], pwm = currentSettings[i]['pwm'])

def HiLevelCommandTurnLeft(sock):
    print('左轉')
    labelBulletin.config(text = '左轉')

    currentSettings = LoadCommandFromFile(home + '/turnleft_settings')
    for i in range(0, 4):
            SendCommandSetAxleAndGetResponse(sock=sock, deviceId=11+i, runDirection = currentSettings[i]['direction'], brake =  currentSettings[i]['brake'], pwm = currentSettings[i]['pwm'])


def HiLevelCommandTurnRight(sock):
    print('右轉')
    labelBulletin.config(text = '右轉')

    currentSettings = LoadCommandFromFile(home + '/turnright_settings')
    for i in range(0, 4):
            SendCommandSetAxleAndGetResponse(sock=sock, deviceId=11+i, runDirection = currentSettings[i]['direction'], brake =  currentSettings[i]['brake'], pwm = currentSettings[i]['pwm'])

def HiLevelCommandNeutral(sock):
    print('N檔')
    labelBulletin.config(text = 'N檔')

    currentSettings = LoadCommandFromFile(home + '/neural_settings')
    for i in range(0, 4):
            SendCommandSetAxleAndGetResponse(sock=sock, deviceId=11+i, runDirection = currentSettings[i]['direction'], brake =  currentSettings[i]['brake'], pwm = currentSettings[i]['pwm'])

def HiLevelCommandBrake(sock):
    print('Parking')
    labelBulletin.config(text = 'Parking')

    currentSettings = LoadCommandFromFile(home + '/brake_settings')
    for i in range(0, 4):
            SendCommandSetAxleAndGetResponse(sock=sock, deviceId=11+i, runDirection = currentSettings[i]['direction'], brake =  currentSettings[i]['brake'], pwm = currentSettings[i]['pwm'])


'''
def HiLevelCommandNeutral(sock):
    print('N檔')

    #with ProcessPoolExecutor(max_workers=40) as executor:
    

    start = time.perf_counter_ns()
    executor.submit(SendCommandSetAxleAndGetResponse, sock=sock, deviceId=11 , brake=True)
    
    #time.sleep(random.random()*TIME_ADDED_BETWEEN_SIGNALS)
    duration2 = time.perf_counter_ns() - start
    executor.submit(SendCommandSetAxleAndGetResponse, sock=sock, deviceId=12 , brake=True)
    
    #time.sleep(random.random()*TIME_ADDED_BETWEEN_SIGNALS)
    duration3 = time.perf_counter_ns() - start
    executor.submit(SendCommandSetAxleAndGetResponse, sock=sock, deviceId=13 , brake=True)
    
    #time.sleep(random.random()*TIME_ADDED_BETWEEN_SIGNALS)
    duration4 = time.perf_counter_ns() - start
    executor.submit(SendCommandSetAxleAndGetResponse, sock=sock, deviceId=14 , brake=True)
    
    print(f"send command at 0 ms, ", duration2/1000000, ' ms, ', duration3/1000000, ' ms, ', duration4/1000000, ' ms')
'''

def SendDataAndGetResponse(sock, data):
    global __controlPermission, __safetyPermission, __safetyOverControl
    if (__controlPermission and (__safetyOverControl if __safetyPermission else True)):
        SendDataAndGetResponse_(sock, data)


def SendCommandSetAxleAndGetResponse(sock, deviceId, runDirection = 0, brake = False, pwm = 0):
    #executor.submit(SendCommandSetAxleAndGetResponse_(sock, deviceId, runDirection, brake, pwm))
    global __controlPermission, __safetyPermission, __safetyOverControl
    if (__controlPermission and (__safetyOverControl if __safetyPermission else True)):
        SendCommandSetAxleAndGetResponse_(sock, deviceId, runDirection, brake, pwm)


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

    #sendOutData.append(0x30)

    #print('send axle command =', sendOutData)

    #receivedData = SendDataAndGetResponse(sock, data=sendOutData)
    SendDataAndGetResponse_(sock, data=sendOutData)
    #print('Axle, received data =', receivedData)
    #return receivedData    

def HiLevelCommnadBreakMotorInitialize(deviceId):
    msg = "剎車初始校正：東方馬達 " + str(deviceId )
    print(msg)
    labelBulletin.config(text = msg)
    sendOutData = [0x69, 0x74, 0x72, 0x69, 0x00, 0x05, 0x01, 0x00, 0x00]
    sendOutData.append(deviceId)
    #print("send out ", sendOutData)
    SendDataAndGetResponse(sock=commandSock, data=sendOutData)

def HiLevelCommnadTurningMotorInitialize(deviceId):
    msg = "轉向初始校正：東方馬達 " + str(deviceId )
    print(msg)
    labelBulletin.config(text = msg)
    sendOutData = [0x69, 0x74, 0x72, 0x69, 0x00, 0x07, 0x01, 0x00, 0x00]
    sendOutData.append(deviceId)
    #print("send out ", sendOutData)
    SendDataAndGetResponse(sock=commandSock, data=sendOutData)

def LoopHandleJoystickEvents(sock):    
    joystickProgram = Joystick()
    joystickProgram.init(partial(HiLevelCommandGoForward, sock), \
        partial(HiLevelCommandGoBackward, sock), \
        partial(HiLevelCommandTurnLeft, sock), \
        partial(HiLevelCommandTurnRight, sock), \
        partial(HiLevelCommandNeutral, sock), \
        partial(HiLevelCommandBrake, sock)), \
        
    joystickProgram.run()

def SaveAllUiStatusToCurrentSettings():
    for i in range(0, 4):
        if uiAxles[i]['toggleBtnForwardBackward']['text'] == '正轉':
            currentSettings[i]['direction'] = 1
        elif uiAxles[i]['toggleBtnForwardBackward']['text'] == '反轉':
            currentSettings[i]['direction'] = 2
        elif uiAxles[i]['toggleBtnForwardBackward']['text'] == '空檔':
            currentSettings[i]['direction'] = 0
        
        if uiAxles[i]['toggleBtnBrake']['text'] == 'Parking false':
            currentSettings[i]['brake'] = 0
        elif uiAxles[i]['toggleBtnBrake']['text'] == 'Parking TRUE':
            currentSettings[i]['brake'] = 1

        currentSettings[i]['pwm'] = uiAxles[i]['comboboxPwm'].get()

    SaveCommandToFile(strFilePath=home + '/current_settings', settings=currentSettings)
        

def ToggleForwardBackbard(id):
    if uiAxles[id - 11]['toggleBtnForwardBackward']['text'] == '正轉':
        uiAxles[id - 11]['toggleBtnForwardBackward'].configure(text = '反轉', bg='pink')
    elif uiAxles[id - 11]['toggleBtnForwardBackward']['text'] == '反轉':
        uiAxles[id - 11]['toggleBtnForwardBackward'].configure(text = '空檔', bg='lightgray')
    elif uiAxles[id - 11]['toggleBtnForwardBackward']['text'] == '空檔':
        uiAxles[id - 11]['toggleBtnForwardBackward'].configure(text = '正轉', bg='lightgreen')
    SaveAllUiStatusToCurrentSettings()

def ToggleBrake(id):
    if uiAxles[id - 11]['toggleBtnBrake']['text'] == 'Parking false':
        uiAxles[id - 11]['toggleBtnBrake'].configure(text = 'Parking TRUE', bg='pink')
    elif uiAxles[id - 11]['toggleBtnBrake']['text'] == 'Parking TRUE':
        uiAxles[id - 11]['toggleBtnBrake'].configure(text = 'Parking false', bg='lightgray')
    SaveAllUiStatusToCurrentSettings()

def PwmCallback(id, event):
    SaveAllUiStatusToCurrentSettings()

breakingValues = [5, 5, 5, 5]
breakingStatus = [False, False, False, False]
def BreakingValueCallback(id, event):
    print(event)
    if id == 1:
        breakingValues[0] = comboboxBreaking1.get()
        print("breakingValue1 = ", breakingValues[0])
    elif id == 2:
        breakingValues[1] = comboboxBreaking2.get()
        print("breakingValue2 = ", breakingValues[1])
    elif id == 3:
        breakingValues[2] = comboboxBreaking3.get()
        print("breakingValue3 = ", breakingValues[2])
    elif id == 4:
        breakingValues[3] = comboboxBreaking4.get()
        print("breakingValue4 = ", breakingValues[3])

def Stop():
    pass

#=================Main UI================
#GUI


def LoadCommandFromFile(strFilePath):
    settingsMotors = [{'direction':0, 'brake':0, 'pwm':28},
                        {'direction':0, 'brake':0, 'pwm':28},
                        {'direction':0, 'brake':0, 'pwm':28},
                        {'direction':0, 'brake':0, 'pwm':28}]
    
    try:
        f=open(strFilePath, 'r')
        settingsInStrIn = f.readlines()  #12 params
        settingsInStr = [x for x in settingsInStrIn if not x.startswith('#')]
        settings = [eval(i) for i in settingsInStr]
        listSettings = list(settings)

        for i in range(0, 4):
            settingsMotors[i]['direction'] = listSettings[i*3]
            settingsMotors[i]['brake'] = listSettings[i*3 + 1]
            settingsMotors[i]['pwm'] = listSettings[i*3 + 2]

            #print('brake = ', settingsMotors[i]['brake'])
        f.close
        return settingsMotors
    except:
        #print('file not exists, use defaults')
        if strFilePath == home + '/forward_settings':
            settingsMotors[0]['direction'] = 2
            settingsMotors[1]['direction'] = 1
            settingsMotors[2]['direction'] = 2
            settingsMotors[3]['direction'] = 1
        elif strFilePath == home + '/backward_settings':
            settingsMotors[0]['direction'] = 1
            settingsMotors[1]['direction'] = 2
            settingsMotors[2]['direction'] = 1
            settingsMotors[3]['direction'] = 2
        elif strFilePath == home + '/turnleft_settings' or strFilePath == home + '/turnleftback_settings':
            settingsMotors[0]['direction'] = 2
            settingsMotors[1]['direction'] = 2
            settingsMotors[2]['direction'] = 2
            settingsMotors[3]['direction'] = 2
        elif strFilePath == home + '/turnright_settings' or strFilePath == home + '/turnrightback_settings':
            settingsMotors[0]['direction'] = 1
            settingsMotors[1]['direction'] = 1
            settingsMotors[2]['direction'] = 1
            settingsMotors[3]['direction'] = 1
        elif strFilePath == home + '/neural_settings':
            settingsMotors[0]['pwm'] = 0
            settingsMotors[1]['pwm'] = 0
            settingsMotors[2]['pwm'] = 0
            settingsMotors[3]['pwm'] = 0
        elif strFilePath == home + '/brake_settings':
            settingsMotors[0]['brake'] = 1
            settingsMotors[1]['brake'] = 1
            settingsMotors[2]['brake'] = 1
            settingsMotors[3]['brake'] = 1
            settingsMotors[0]['pwm'] = 0
            settingsMotors[1]['pwm'] = 0
            settingsMotors[2]['pwm'] = 0
            settingsMotors[3]['pwm'] = 0
        
        '''
        Path("/tmp/").mkdir(parents=True, exist_ok=True)

        f1=open(strFilePath, 'a+')
        for i in range(0, 4):
            directionInBytes = bytearray(struct.pack("i", (int)(settingsMotors[i]['direction'])))
            f1.write(directionInBytes)

            brakeInBytes = bytearray(struct.pack("i", (int)(settingsMotors[i]['brake'])))
            f1.write(brakeInBytes)

            pwmInBytes = bytearray(struct.pack("i", (int) (settingsMotors[i]['pwm'])))
            f1.write(pwmInBytes)
        f1.close
        '''
        
        
        
    
        return settingsMotors
        
def SaveCommandToFile(strFilePath, settings):
    try:
        f = open(strFilePath, 'w+')
        #print('save ', strFilePath)

        #for i in range(0,4):
        #    print('save settings to ', strFilePath, ': direction = ', settings[i]['direction'], ', pwm = ', settings[i]['pwm'], ', brake = ', settings[i]['brake'])

        #print('-------SAVE------')
        for i in range(0, 4):
            iHumanRead = str(i + 1)
            dataToWrite = ['#-----Motor ' + iHumanRead + ', Direction' + '\n', \
                str((int)(settings[i]['direction'])) + '\n', \
                '#-----         ' + 'Brake' + '\n', \
                str((int)(settings[i]['brake'])) + '\n', \
                '#-----         ' + 'PWM' + '\n', \
                str((int)(settings[i]['pwm'])) + '\n' \
            ]            
            f.writelines(dataToWrite)
            
            print(dataToWrite)
        print('\n')
        
        f.close
        
    except:
        print('cannot save ', strFilePath)
        pass

def GetMotorDistanceAccordingToTurningDegree(degree):
    distance = (5 + degree*0.25)*1000
    distance = int(distance)
    return distance

headingDegrees = [0, 0, 0, 0]
PLUS_OR_MINUS_STEP = 4
def HiLevelCommnadHeadingChange(degreeChange, deviceId):
    sendOutData = [0x69, 0x74, 0x72, 0x69, 0x00, 0x08, 0x01, 0x00, 0x02]
    distance = 0
    msg = ""
    if deviceId == 41:
        headingDegrees[0] += degreeChange
        if headingDegrees[0] > 20:
            headingDegrees[0] = 20
        elif headingDegrees[0] < -20:
            headingDegrees[0] = -20

        labelHeadingCurrentAngle1.config(text = headingDegrees[0])
        msg = "轉向角度：" + str(headingDegrees[0]) + " 東方馬達 " + str(deviceId )
        distance = GetMotorDistanceAccordingToTurningDegree(headingDegrees[0])
    elif deviceId == 42:
        headingDegrees[1] += degreeChange
        if headingDegrees[1] > 20:
            headingDegrees[1] = 20
        elif headingDegrees[1] < -20:
            headingDegrees[1] = -20
    
        labelHeadingCurrentAngle2.config(text = headingDegrees[1])
        msg = "轉向角度：" + str(headingDegrees[1]) + " 東方馬達 " + str(deviceId )
        distance = GetMotorDistanceAccordingToTurningDegree(headingDegrees[1])
    elif deviceId == 43:
        headingDegrees[2] += degreeChange
        if headingDegrees[2] > 20:
            headingDegrees[2] = 20
        elif headingDegrees[2] < -20:
            headingDegrees[2] = -20

        labelHeadingCurrentAngle3.config(text = headingDegrees[2])
        msg = "轉向角度：" + str(headingDegrees[2]) + " 東方馬達 " + str(deviceId )
        distance = GetMotorDistanceAccordingToTurningDegree(headingDegrees[2])
    elif deviceId == 44:
        headingDegrees[3] += degreeChange
        if headingDegrees[3] > 20:
            headingDegrees[3] = 20
        elif headingDegrees[3] < -20:
            headingDegrees[3] = -20

        labelHeadingCurrentAngle4.config(text = headingDegrees[3])
        msg = "轉向角度：" + str(headingDegrees[3]) + " 東方馬達 " + str(deviceId )
        distance = GetMotorDistanceAccordingToTurningDegree(headingDegrees[3])

    print("distance = ", distance)
    distanceLo = (int)(distance%256)
    distanceHi = (int)(distance/256)
    sendOutData.append(distanceHi)
    sendOutData.append(distanceLo)
    sendOutData.append(deviceId)
    print("send out ", sendOutData)
    SendDataAndGetResponse(sock=commandSock, data=sendOutData)

    print(msg)
    labelBulletin.config(text = msg)
        


def HiLevelCommnadHeadingZero(deviceId):
    if deviceId == 41:
        headingDegrees[0] = 0
        labelHeadingCurrentAngle1.config(text = headingDegrees[0])        
    elif deviceId == 42:
        headingDegrees[1] = 0
        labelHeadingCurrentAngle2.config(text = headingDegrees[1])
    elif deviceId == 43:
        headingDegrees[2] = 0
        labelHeadingCurrentAngle3.config(text = headingDegrees[2])
    elif deviceId == 44:
        headingDegrees[3] = 0
        labelHeadingCurrentAngle4.config(text = headingDegrees[3])

    msg = "轉向角度：0", " 東方馬達 " + str(deviceId )
    print(msg)
    labelBulletin.config(text = msg)
    sendOutData = [0x69, 0x74, 0x72, 0x69, 0x00, 0x08, 0x01, 0x00, 0x02]
    distance = GetMotorDistanceAccordingToTurningDegree(0)
    print("distance = ", distance)
    distanceLo = (int)(distance%256)
    distanceHi = (int)(distance/256)
    sendOutData.append(distanceHi)
    sendOutData.append(distanceLo)
    sendOutData.append(deviceId)
    print("send out ", sendOutData)
    SendDataAndGetResponse(sock=commandSock, data=sendOutData)

def HiLevelCommnadBreak(deviceId):
    sendOutData = [0x69, 0x74, 0x72, 0x69, 0x00, 0x06, 0x01, 0x00, 0x02]
    if breakingStatus[deviceId - 1] == True:# no breaking ON
        sendOutData.append(0x00)
        sendOutData.append(0x00)
        breakingStatus[deviceId - 1] = False
        if deviceId == 1:
            labelBreakingCurrentState1.config(text = "Now Off")
        elif deviceId == 2:
            labelBreakingCurrentState2.config(text = "Now Off")
        elif deviceId == 3:
            labelBreakingCurrentState3.config(text = "Now Off")
        elif deviceId == 4:
            labelBreakingCurrentState4.config(text = "Now Off")
    elif breakingStatus[deviceId - 1] == False:# no breaking OFF
        distance = int(breakingValues[deviceId -1]) * 1000
    
        distanceLo = (int)(distance%256)
        distanceHi = (int)(distance/256)
        sendOutData.append(distanceHi)
        sendOutData.append(distanceLo)
        breakingStatus[deviceId - 1] = True
        if deviceId == 1:
            labelBreakingCurrentState1.config(text = "Now On")
        elif deviceId == 2:
            labelBreakingCurrentState2.config(text = "Now On")
        elif deviceId == 3:
            labelBreakingCurrentState3.config(text = "Now On")
        elif deviceId == 4:
            labelBreakingCurrentState4.config(text = "Now On")
        
    sendOutData.append(deviceId)
    #print("send out ", sendOutData)
    SendDataAndGetResponse(sock=commandSock, data=sendOutData)

currentSettings = LoadCommandFromFile(home + '/current_settings')

def SaveCommandForward():
    print('目前設置 --> “前進” 指令')
    labelBulletin.config(text = '目前設置 --> “前進” 指令')
    currentSettings = LoadCommandFromFile(home + '/current_settings')
    SaveCommandToFile(strFilePath=home + '//forward_settings', settings=currentSettings)
    

def LoadCommandForward():
    print('“前進” 指令 --> 目前設置')
    labelBulletin.config(text = '目前設置 --> “前進” 指令')
    currentSettings = LoadCommandFromFile(home + '/forward_settings')
    SaveCommandToFile(strFilePath=home + 'current_settings', settings=currentSettings)
    for i in range(0,4):
        print('current settings: direction = ', currentSettings[i]['direction'], ', pwm = ', currentSettings[i]['pwm'], ',brake = ', currentSettings[i]['brake'])
    InitialUIAccordingToSettings(currentSettings)


def SaveCommandBackward():
    print('目前設置 --> “後退” 指令')
    labelBulletin.config(text = '目前設置 --> “後退” 指令')
    currentSettings = LoadCommandFromFile(home + '/current_settings')
    SaveCommandToFile(strFilePath=home + '/backward_settings', settings=currentSettings)

def LoadCommandBackward():
    print('“後退” 指令 --> 目前設置')
    labelBulletin.config(text = '“後退” 指令 --> 目前設置')
    currentSettings = LoadCommandFromFile(home + '/backward_settings')
    SaveCommandToFile(strFilePath=home + '/current_settings', settings=currentSettings)
    for i in range(0,4):
        print('current settings: direction = ', currentSettings[i]['direction'], ', pwm = ', currentSettings[i]['pwm'], ',brake = ', currentSettings[i]['brake'])
    InitialUIAccordingToSettings(currentSettings)

def SaveCommandTuenLeft():
    print('目前設置 --> “左轉” 指令')
    labelBulletin.config(text = '目前設置 --> “左轉” 指令')
    currentSettings = LoadCommandFromFile(home + '/current_settings')
    SaveCommandToFile(strFilePath=home + '/turnleft_settings', settings=currentSettings)

def SaveCommandTuenLeftBack():
    print('目前設置 --> “左後轉” 指令')
    labelBulletin.config(text = '目前設置 --> “左後轉” 指令')
    currentSettings = LoadCommandFromFile(home + '/current_settings')
    SaveCommandToFile(strFilePath=home + '/turnleftback_settings', settings=currentSettings)

def LoadCommandTurnLeft():
    print('“左轉” 指令 --> 目前設置')
    labelBulletin.config(text = '“左轉” 指令 --> 目前設置')
    currentSettings = LoadCommandFromFile(home + '/turnleft_settings')
    SaveCommandToFile(strFilePath=home + '/current_settings', settings=currentSettings)
    for i in range(0,4):
        print('current settings: direction = ', currentSettings[i]['direction'], ', pwm = ', currentSettings[i]['pwm'], ', brake = ', currentSettings[i]['brake'])
    InitialUIAccordingToSettings(currentSettings)

def LoadCommandTurnLeftBack():
    print('“左後轉” 指令 --> 目前設置')
    labelBulletin.config(text = '“左後轉” 指令 --> 目前設置')
    currentSettings = LoadCommandFromFile(home + '/turnleftback_settings')
    SaveCommandToFile(strFilePath=home + '/current_settings', settings=currentSettings)
    for i in range(0,4):
        print('current settings: direction = ', currentSettings[i]['direction'], ', pwm = ', currentSettings[i]['pwm'], ', brake = ', currentSettings[i]['brake'])
    InitialUIAccordingToSettings(currentSettings)

def SaveCommandTuenRight():
    print('目前設置 --> “右轉” 指令')
    labelBulletin.config(text = '目前設置 --> “右轉” 指令')
    currentSettings = LoadCommandFromFile(home + '/current_settings')
    SaveCommandToFile(strFilePath=home + '/turnright_settings', settings=currentSettings)

def SaveCommandTuenRightBack():
    print('目前設置 --> “右後轉” 指令')
    labelBulletin.config(text = '目前設置 --> “右後轉” 指令')
    currentSettings = LoadCommandFromFile(home + '/current_settings')
    SaveCommandToFile(strFilePath=home + '/turnrightback_settings', settings=currentSettings)

def LoadCommandTurnRight():
    print('“右轉” 指令 --> 目前設置')
    labelBulletin.config(text = '“右轉” 指令 --> 目前設置')
    currentSettings = LoadCommandFromFile(home + '/turnright_settings')
    SaveCommandToFile(strFilePath=home + '/current_settings', settings=currentSettings)
    for i in range(0,4):
        print('current settings: direction = ', currentSettings[i]['direction'], ', pwm = ', currentSettings[i]['pwm'], ', brake = ', currentSettings[i]['brake'])
    InitialUIAccordingToSettings(currentSettings)

def LoadCommandTurnRightBack():
    print('“右後轉” 指令 --> 目前設置')
    labelBulletin.config(text = '“右後轉” 指令 --> 目前設置')
    currentSettings = LoadCommandFromFile(home + '/turnrightback_settings')
    SaveCommandToFile(strFilePath=home + '/current_settings', settings=currentSettings)
    for i in range(0,4):
        print('current settings: direction = ', currentSettings[i]['direction'], ', pwm = ', currentSettings[i]['pwm'], ', brake = ', currentSettings[i]['brake'])
    InitialUIAccordingToSettings(currentSettings)
    
def SaveCommandNeural():
    print('目前設置 --> “空檔” 指令')
    labelBulletin.config(text = '目前設置 --> “空檔” 指令')
    currentSettings = LoadCommandFromFile(home + '/current_settings')
    SaveCommandToFile(strFilePath=home + '/neural_settings', settings=currentSettings)
    for i in range(0,4):
        print('current settings: direction = ', currentSettings[i]['direction'], ', pwm = ', currentSettings[i]['pwm'], ', brake = ', currentSettings[i]['brake'])

def LoadCommandNeural():
    print('“空檔” 指令 --> 目前設置')
    labelBulletin.config(text = '“空檔” 指令 --> 目前設置')
    currentSettings = LoadCommandFromFile(home + '/neural_settings')
    SaveCommandToFile(strFilePath=home + '/current_settings', settings=currentSettings)
    for i in range(0,4):
        print('current settings: direction = ', currentSettings[i]['direction'], ', pwm = ', currentSettings[i]['pwm'], ', brake = ', currentSettings[i]['brake'])
    InitialUIAccordingToSettings(currentSettings)

def SaveCommandBrake():
    print('目前設置 --> “Parking” 指令')
    labelBulletin.config(text = '目前設置 --> “Parking” 指令')
    currentSettings = LoadCommandFromFile(home + '/current_settings')
    SaveCommandToFile(strFilePath=home + '/brake_settings', settings=currentSettings)

def LoadCommandBrake():
    print('“Parking” 指令 --> 目前設置')
    labelBulletin.config(text = '“Parking” 指令 --> 目前設置')
    currentSettings = LoadCommandFromFile(home + '/brake_settings')
    SaveCommandToFile(strFilePath=home + '/current_settings', settings=currentSettings)
    for i in range(0,4):
        print('current settings: direction = ', currentSettings[i]['direction'], ', pwm = ', currentSettings[i]['pwm'], ', brake = ', currentSettings[i]['brake'])
    InitialUIAccordingToSettings(currentSettings)

def InitialUIAccordingToSettings(settings):
    for i in range(0, 4):
        if settings[i]['direction'] == 2:
            uiAxles[i]['toggleBtnForwardBackward'].configure(text = '反轉', bg='pink')
        elif settings[i]['direction'] == 1:
            uiAxles[i]['toggleBtnForwardBackward'].configure(text = '正轉', bg='lightgreen')
        elif settings[i]['direction'] == 0:
            uiAxles[i]['toggleBtnForwardBackward'].configure(text = '空檔', bg='lightgray')

        uiAxles[i]['comboboxPwm'].set(settings[i]['pwm'])

        if settings[i]['brake'] == 1:
            uiAxles[i]['toggleBtnBrake'].configure(text = 'Parking TRUE', bg='pink')
        elif settings[i]['brake'] == 0:
            uiAxles[i]['toggleBtnBrake'].configure(text = 'Parking false', bg='lightgray')

    

if __name__ == '__main__':
    #=================Main UI================
    app = tk.Tk()
    app.title('獨立輪車通訊網路 Control-1 Client')
    app.geometry('1500x800')

    uiAxles = []
    for id in range(11,15):
        textName = '輪軸馬達'+ str(id-10)
        name = tk.StringVar()
        name.set(textName)
        labelName = tk.Label(app, textvariable=name)
        labelName.place(x = (id-11)*200 + 50, y = 1)
            
        toggleBtnForwardBackward = tk.Button(text="正轉", bg='lightgreen', width=10, relief='raised', command = partial(ToggleForwardBackbard, id))
        toggleBtnForwardBackward.place(x = (id-11)*200 + 50, y = 50)

        comboboxPwm = ttk.Combobox(app, values=[i for i in range(0,101)], width = 10)
        comboboxPwm.set(28)
        comboboxPwm.bind('<<ComboboxSelected>>', partial(PwmCallback, id))    
        comboboxPwm.place(x = (id-11)*200 + 50, y = 50*2)

        toggleBtnBrake = tk.Button(text="Parking false", bg='lightgray', width=10, relief='raised', command = partial(ToggleBrake, id))
        toggleBtnBrake.place(x = (id-11)*200 + 50, y = 50*3)

        
        newUiAxle={'id':id, #11-19
                    'name':name,
                    'labelName':labelName,
                    'toggleBtnForwardBackward':toggleBtnForwardBackward,
                    'comboboxPwm':comboboxPwm,
                    'toggleBtnBrake':toggleBtnBrake,
                    }
        uiAxles.append(newUiAxle)

    btnLaunch = tk.Button(text="送出指令", bg='lightgreen', width=10, height = 5, relief='raised', command = HiLevelCommandCurrent)
    btnLaunch.place(x = 850, y = 70)

    btnStop = tk.Button(text="停止", bg='pink', width=10, height = 5, relief='raised', command = HiLevelCommandBrakeAll)
    btnStop.place(x = 1000, y = 70)

    ttk.Separator(app).place(x=1,y=200, relwidth=1)
        
    btnLoadForward = tk.Button(text="‘前進’讀檔", bg='lightblue', width=10, height = 3, relief='raised', command = LoadCommandForward)
    btnLoadForward.place(x = 50, y = 220)

    btnLoadBackward = tk.Button(text="‘後退’讀檔", bg='lightblue', width=10, height = 3, relief='raised', command = LoadCommandBackward)
    btnLoadBackward.place(x = 170+ 50, y = 220)

    btnLoadTurnLeft = tk.Button(text="‘左轉’讀檔", bg='lightblue', width=10, height = 3, relief='raised', command = LoadCommandTurnLeft)
    btnLoadTurnLeft.place(x = 170 * 2 + 50, y = 220)

    btnLoadTurnRight = tk.Button(text="‘右轉’讀檔", bg='lightblue', width=10, height = 3, relief='raised', command = LoadCommandTurnRight)
    btnLoadTurnRight.place(x = 170 * 3 + 50, y = 220)

    btnLoadNeutral = tk.Button(text="‘空檔’讀檔", bg='lightblue', width=10, height = 3, relief='raised', command = LoadCommandNeural)
    btnLoadNeutral.place(x = 170 * 4 + 50, y = 220)

    btnLoadBrake = tk.Button(text="‘Parking’讀檔", bg='lightblue', width=10, height = 3, relief='raised', command = LoadCommandBrake)
    btnLoadBrake.place(x = 170 * 5 + 50, y = 220)

    btnLoadTurnLeftBack = tk.Button(text="‘左後轉’讀檔", bg='lightblue', width=10, height = 3, relief='raised', command = LoadCommandTurnLeftBack)
    btnLoadTurnLeftBack.place(x = 170 * 6 + 50, y = 220)

    btnLoadTurnRightBack = tk.Button(text="‘右後轉’讀檔", bg='lightblue', width=10, height = 3, relief='raised', command = LoadCommandTurnRightBack)
    btnLoadTurnRightBack.place(x = 170 * 7 + 50, y = 220)


    btnSaveForward = tk.Button(text="‘前進’存檔", bg='pink', width=10, height = 3, relief='raised', command = SaveCommandForward)
    btnSaveForward.place(x = 50, y = 310)

    btnSaveBackward = tk.Button(text="‘後退’存檔", bg='pink', width=10, height = 3, relief='raised', command = SaveCommandBackward)
    btnSaveBackward.place(x = 170 * 1 + 50, y = 310)

    btnSaveTurnLeft = tk.Button(text="‘左轉’存檔", bg='pink', width=10, height = 3, relief='raised', command = SaveCommandTuenLeft)
    btnSaveTurnLeft.place(x = 170 * 2 + 50, y = 310)

    btnSaveTurnRight = tk.Button(text="‘右轉’存檔", bg='pink', width=10, height = 3, relief='raised', command = SaveCommandTuenRight)
    btnSaveTurnRight.place(x = 170 * 3 + 50, y = 310)

    btnSaveNeutral = tk.Button(text="‘空檔’存檔", bg='pink', width=10, height = 3, relief='raised', command = SaveCommandNeural)
    btnSaveNeutral.place(x = 170 * 4 + 50, y = 310)

    btnSaveBrake = tk.Button(text="‘Parking’存檔", bg='pink', width=10, height = 3, relief='raised', command = SaveCommandBrake)
    btnSaveBrake.place(x = 170 * 5 + 50, y = 310)

    btnSaveTurnLeft = tk.Button(text="‘左後轉’存檔", bg='pink', width=10, height = 3, relief='raised', command = SaveCommandTuenLeftBack)
    btnSaveTurnLeft.place(x = 170 * 6 + 50, y = 310)

    btnSaveTurnRight = tk.Button(text="‘右後轉’存檔", bg='pink', width=10, height = 3, relief='raised', command = SaveCommandTuenRightBack)
    btnSaveTurnRight.place(x = 170 * 7 + 50, y = 310)

    ttk.Separator(app).place(x=1,y=400, relwidth=1)

    #===========輪1===================
    tk.LabelFrame(app, width = 330, height= 300, borderwidth=3).place(x = 10, y = 410)
    tk.Label(app, text='輪1', font=("Arial", 18)).place(x = 160, y = 420)
    tk.Label(app, text='任務一', font=("Arial", 13)).place(x = 80, y = 450)
    tk.Label(app, text='（轉向）', font=("Arial", 13)).place(x = 80, y = 470)
    tk.Label(app, text='任務二', font=("Arial", 13)).place(x = 240, y = 450)
    tk.Label(app, text='（剎車）', font=("Arial", 13)).place(x = 240, y = 470)

    btnHeadingInitial1 = tk.Button(text="初始校正", bg='lightgreen', width=18, height = 2, relief='raised', command = partial(HiLevelCommnadTurningMotorInitialize, 41))
    btnHeadingInitial1.place(x = 20, y = 500)
    tk.Label(app, text='目前\n角度').place(x = 85, y = 550)
    labelHeadingCurrentAngle1 = tk.Label(app, text='0', font=("Arial", 25))
    labelHeadingCurrentAngle1.place(x = 120, y = 550)
    labelBulletin = tk.Label(app, text='Status')
    btnHeadingMinus1 = tk.Button(text="反轉\n增益", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingChange, -PLUS_OR_MINUS_STEP, 41))
    btnHeadingMinus1.place(x = 20, y = 590)
    btnHeadingZero1 = tk.Button(text="0\n直行", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingZero, 41))
    btnHeadingZero1.place(x = 80, y = 590)
    btnHeadingPlus1 = tk.Button(text="正轉\n增益", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingChange, PLUS_OR_MINUS_STEP, 41))
    btnHeadingPlus1.place(x = 140, y = 590)

    btnBreakingInitial1 = tk.Button(text="初始校正", bg='lightgreen', width=10, height = 2, relief='raised', command = partial(HiLevelCommnadBreakMotorInitialize, 1))
    btnBreakingInitial1.place(x = 220, y = 500)
    comboboxBreaking1 = ttk.Combobox(app, values=[i for i in range(0,11)], width = 3, font=("Arial", 16))
    comboboxBreaking1.set(5)
    comboboxBreaking1.bind('<<ComboboxSelected>>', partial(BreakingValueCallback, 1))
    comboboxBreaking1.place(x = 240, y = 550)
    btnBreaking1 = tk.Button(text="ON/OFF", bg='lightgreen', width=10, height = 2, relief='raised', command = partial(HiLevelCommnadBreak, 1))
    btnBreaking1.place(x = 220, y = 590)
    labelBreakingCurrentState1 = tk.Label(app, text='Now Off', font=("Arial", 13))
    labelBreakingCurrentState1.place(x = 220, y = 640)

    #===========輪2===================
    tk.LabelFrame(app, width = 310, height= 300, borderwidth=3).place(x = 350, y = 410)
    tk.Label(app, text='輪2', font=("Arial", 18)).place(x = 480, y = 420)
    tk.Label(app, text='任務一', font=("Arial", 13)).place(x = 400, y = 450)
    tk.Label(app, text='（轉向）', font=("Arial", 13)).place(x = 400, y = 470)
    tk.Label(app, text='任務二', font=("Arial", 13)).place(x = 560, y = 450)
    tk.Label(app, text='（剎車）', font=("Arial", 13)).place(x = 560, y = 470)

    btnHeadingInitial2 = tk.Button(text="初始校正", bg='lightgreen', width=18, height = 2, relief='raised', command = partial(HiLevelCommnadBreakMotorInitialize, 42))
    btnHeadingInitial2.place(x = 360, y = 500)
    tk.Label(app, text='目前\n角度').place(x = 405, y = 550)
    labelHeadingCurrentAngle2 = tk.Label(app, text='0', font=("Arial", 25))
    labelHeadingCurrentAngle2.place(x = 440, y = 550)
    btnHeadingMinus2 = tk.Button(text="反轉\n增益", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingChange, -PLUS_OR_MINUS_STEP, 42))
    btnHeadingMinus2.place(x = 360, y = 590)
    btnHeadingZero2 = tk.Button(text="0\n直行", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingZero, 42))
    btnHeadingZero2.place(x = 420, y = 590)
    btnHeadingPlus2 = tk.Button(text="正轉\n增益", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingChange, PLUS_OR_MINUS_STEP, 42))
    btnHeadingPlus2.place(x = 480, y = 590)

    btnBreakingInitial2 = tk.Button(text="初始校正", bg='lightgreen', width=10, height = 2, relief='raised', command = partial(HiLevelCommnadBreakMotorInitialize, 2))
    btnBreakingInitial2.place(x = 540, y = 500)
    comboboxBreaking2 = ttk.Combobox(app, values=[i for i in range(0,11)], width = 3, font=("Arial", 16))
    comboboxBreaking2.set(5)
    comboboxBreaking2.bind('<<ComboboxSelected>>', partial(BreakingValueCallback, 2))    
    comboboxBreaking2.place(x = 560, y = 550)
    btnBreaking2 = tk.Button(text="ON/OFF", bg='lightgreen', width=10, height = 2, relief='raised', command = partial(HiLevelCommnadBreak, 2))
    btnBreaking2.place(x = 540, y = 590)
    labelBreakingCurrentState2 = tk.Label(app, text='Now Off', font=("Arial", 13))
    labelBreakingCurrentState2.place(x = 540, y = 640)


    #===========輪3===================
    tk.LabelFrame(app, width = 310, height= 300, borderwidth=3).place(x = 670, y = 410)
    tk.Label(app, text='輪3', font=("Arial", 18)).place(x = 800, y = 420)
    tk.Label(app, text='任務一', font=("Arial", 13)).place(x = 720, y = 450)
    tk.Label(app, text='（轉向）', font=("Arial", 13)).place(x = 720, y = 470)
    tk.Label(app, text='任務二', font=("Arial", 13)).place(x = 880, y = 450)
    tk.Label(app, text='（剎車）', font=("Arial", 13)).place(x = 880, y = 470)

    btnHeadingInitial3 = tk.Button(text="初始校正", bg='lightgreen', width=18, height = 2, relief='raised', command = partial(HiLevelCommnadBreakMotorInitialize, 43))
    btnHeadingInitial3.place(x = 680, y = 500)
    tk.Label(app, text='目前\n角度').place(x = 725, y = 550)
    labelHeadingCurrentAngle3= tk.Label(app, text='0', font=("Arial", 25))
    labelHeadingCurrentAngle3.place(x = 760, y = 550)
    btnHeadingMinus3 = tk.Button(text="反轉\n增益", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingChange, -PLUS_OR_MINUS_STEP, 43))
    btnHeadingMinus3.place(x = 680, y = 590)
    btnHeadingZero3= tk.Button(text="0\n直行", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingZero, 43))
    btnHeadingZero3.place(x = 740, y = 590)
    btnHeadingPlus3= tk.Button(text="正轉\n增益", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingChange, PLUS_OR_MINUS_STEP, 43))
    btnHeadingPlus3.place(x = 800, y = 590)

    btnBreakingInitial3 = tk.Button(text="初始校正", bg='lightgreen', width=10, height = 2, relief='raised', command = partial(HiLevelCommnadBreakMotorInitialize, 3))
    btnBreakingInitial3.place(x = 860, y = 500)
    comboboxBreaking3 = ttk.Combobox(app, values=[i for i in range(0,11)], width = 3, font=("Arial", 16))
    comboboxBreaking3.set(5)
    comboboxBreaking3.bind('<<ComboboxSelected>>', partial(BreakingValueCallback, 3))    
    comboboxBreaking3.place(x = 880, y = 550)
    btnBreaking3 = tk.Button(text="ON/OFF", bg='lightgreen', width=10, height = 2, relief='raised', command = partial(HiLevelCommnadBreak, 3))
    btnBreaking3.place(x = 860, y = 590)
    labelBreakingCurrentState3 = tk.Label(app, text='Now Off', font=("Arial", 13))
    labelBreakingCurrentState3.place(x = 860, y = 640)


    #===========輪4===================
    tk.LabelFrame(app, width = 310, height= 300, borderwidth=3).place(x = 990, y = 410)
    tk.Label(app, text='輪4', font=("Arial", 18)).place(x = 1120, y = 420)
    tk.Label(app, text='任務一', font=("Arial", 13)).place(x = 1040, y = 450)
    tk.Label(app, text='（轉向）', font=("Arial", 13)).place(x = 1040, y = 470)
    tk.Label(app, text='任務二', font=("Arial", 13)).place(x = 1200, y = 450)
    tk.Label(app, text='（剎車）', font=("Arial", 13)).place(x = 1200, y = 470)

    btnHeadingInitial4 = tk.Button(text="初始校正", bg='lightgreen', width=18, height = 2, relief='raised', command = partial(HiLevelCommnadBreakMotorInitialize, 44))
    btnHeadingInitial4.place(x = 1000, y = 500)
    tk.Label(app, text='目前\n角度').place(x = 1045, y = 550)
    labelHeadingCurrentAngle4 = tk.Label(app, text='0', font=("Arial", 25))
    labelHeadingCurrentAngle4.place(x = 1080, y = 550)

    btnHeadingMinus4 = tk.Button(text="反轉\n增益", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingChange, -PLUS_OR_MINUS_STEP, 44))
    btnHeadingMinus4.place(x = 1000, y = 590)
    btnHeadingZero4 = tk.Button(text="0\n直行", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingZero, 44))
    btnHeadingZero4.place(x = 1060, y = 590)
    btnHeadingPlus4 = tk.Button(text="正轉\n增益", bg='lightgreen', width=3, height = 2, relief='raised', command = partial(HiLevelCommnadHeadingChange, PLUS_OR_MINUS_STEP, 44))
    btnHeadingPlus4.place(x = 1120, y = 590)

    btnBreakingInitial4 = tk.Button(text="初始校正", bg='lightgreen', width=10, height = 2, relief='raised', command = partial(HiLevelCommnadBreakMotorInitialize, 4))
    btnBreakingInitial4.place(x = 1180, y = 500)
    comboboxBreaking4 = ttk.Combobox(app, values=[i for i in range(0,11)], width = 3, font=("Arial", 16))
    comboboxBreaking4.set(5)
    comboboxBreaking4.bind('<<ComboboxSelected>>', partial(BreakingValueCallback, 4))    
    comboboxBreaking4.place(x = 1200, y = 550)
    btnBreaking4 = tk.Button(text="ON/OFF", bg='lightgreen', width=10, height = 2, relief='raised', command = partial(HiLevelCommnadBreak, 4))
    btnBreaking4.place(x = 1180, y = 590)
    labelBreakingCurrentState4 = tk.Label(app, text='Now Off', font=("Arial", 13))
    labelBreakingCurrentState4.place(x = 1180, y = 640)

    labelBulletin = tk.Label(app, text='Status')
    labelBulletin.place(x = 50, y = 720)

    #set initial values

    InitialUIAccordingToSettings(currentSettings)

    time.sleep(1)

    myID = 0

    dataSock = sockConnect(serverIp)
    sendCommandTellId(dataSock)

    imAliveSock = sockConnectImAlive(serverIp)    
    receivedData = sendCommandTellId(imAliveSock)

    handler1 = Thread(target=LoopSendAlive, args=(imAliveSock, ))
    handler1.start()


    commandSock = sockConnectCommand(serverIp)


    handler2 = Thread(target=LoopHandleJoystickEvents, args=(commandSock, ))
    handler2.start()

    app.mainloop()



    '''
    while True:
        HiLevelCommandGoForward(commandSock)
        time.sleep(5)
        HiLevelCommandTurnLeft(commandSock)
        time.sleep(5)
        HiLevelCommandTurnRight(commandSock)
        time.sleep(5)
        HiLevelCommandNeutral(commandSock)
        time.sleep(2)
        HiLevelCommandGoBackward(commandSock)
        time.sleep(5)
        HiLevelCommandNeutral(commandSock)
        time.sleep(2)
    '''
