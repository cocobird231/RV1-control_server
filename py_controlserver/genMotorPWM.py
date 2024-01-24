import control
import numpy as np
import math

def ValueMapping(value, from_start, from_end, to_start, to_end):
    a = (to_end - to_start) / (from_end - from_start)

    if (a < 0):
        return a * (value - from_start) + to_start
    else:
        return a * (value - from_start) + to_start

def GammaCorrection(value, gamma, min_bound = 0, max_bound = 1):
    assert(value >= min_bound and value <= max_bound and max_bound > min_bound)
    if (min_bound == 0 and max_bound == 1):
        return value**gamma
    ratio = (value - min_bound) / (max_bound - min_bound)
    return ratio**gamma * (max_bound - min_bound) + min_bound

################################################################################################
#                                       Chassis Parameters
################################################################################################
STEERINGWHEEL_TOLERANCE = 1500# left < -1500, 1500 > right
CAR_LENGTH = 2200# unit: mm
CAR_WIDTH = 2080# unit: mm
MAX_RADIUS_ACKERMANN = 60000# unit: mm
MIN_RADIUS_ACKERMANN = 7100# unit: mm
MAX_RADIUS_4WS = 30000# unit: mm
MIN_RADIUS_4WS = 4100# unit: mm
MIN_WHEEL_STEER_ANGLE = 0
MAX_WHEEL_STEER_ANGLE = 20

M1_OFFSET = (1040, 1100)
M2_OFFSET = (-1040, 1100)
M3_OFFSET = (1040, -1100)
M4_OFFSET = (-1040, -1100)


################################################################################################
#                                       Motor Parameter Definitions
################################################################################################
motorDirectionCorrectionList = [1, 1, 1, 1]
limitRPM = 500
limitPWM = 40
motorIDList = [11, 12, 13, 14]
steeringIDList = [41, 42, 43, 44]

m1_y = np.array([9, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55])
m2_y = np.array([9, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55])
m3_y = np.array([9, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55])
m4_y = np.array([9, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55])

m1_x = np.array([10.008, 18.782, 51.208, 71.842, 86.204, 97.57, 106.33, 113.8, 120.27, 125.74, 132.48])
m2_x = np.array([19.38, 27.645, 57.235, 75.871, 89.3, 100.18, 108.56, 115.94, 121.29, 127.41, 128.53])
m3_x = np.array([13.405, 22.36, 53.677, 73.156, 87.342, 97.856, 107.11, 114.19, 119.71, 126.05, 127.83])
m4_x = np.array([23.64, 31.071, 58.909, 76.695, 89.88, 99.81, 108.37, 115.45, 120.98, 127.67, 128.28])

p1 = np.polyfit(m1_x, m1_y, 3)  # Last argument is degree of polynomial
p2 = np.polyfit(m2_x, m2_y, 3)  # Last argument is degree of polynomial
p3 = np.polyfit(m3_x, m3_y, 3)  # Last argument is degree of polynomial
p4 = np.polyfit(m4_x, m4_y, 3)  # Last argument is degree of polynomial

predict1 = np.poly1d(p1)
predict2 = np.poly1d(p2)
predict3 = np.poly1d(p3)
predict4 = np.poly1d(p4)


def MotorRPMToPWM(m1, m2, m3, m4, pwmLimit = 70, pwmMin = 0):# Unloaded test parameters
    predPWM1 = predict1(m1)
    predPWM2 = predict2(m2)
    predPWM3 = predict3(m3)
    predPWM4 = predict4(m4)
    predPWM1 = 0 if (predPWM1 < pwmMin) else (pwmLimit if (predPWM1 > pwmLimit) else predPWM1)
    predPWM2 = 0 if (predPWM2 < pwmMin) else (pwmLimit if (predPWM2 > pwmLimit) else predPWM2)
    predPWM3 = 0 if (predPWM3 < pwmMin) else (pwmLimit if (predPWM3 > pwmLimit) else predPWM3)
    predPWM4 = 0 if (predPWM4 < pwmMin) else (pwmLimit if (predPWM4 > pwmLimit) else predPWM4)
    return [int(predPWM1), int(predPWM2), int(predPWM3), int(predPWM4)]


################################################################################################
#                                       Steering Method Definitions
################################################################################################
steeringCorrectionList = [-1, 1, -1, 1]# turn right if angle > 0
''' Steering Method in Low Speed

steering direction correction [-1, 1, -1, 1]
                ^
   [12] / /-|========|-/ / [11]
            |========|
            |========|
   [14] \ \-|========|-\ \ [13]

                ^
   [12] \ \-|========|-\ \ [11]
            |========|
            |========|
   [14] / /-|========|-/ / [13]



steering direction correction [-1, 1, -1, 1] (turn right if angle > 0)
                ^
   [13] / /-|========|-/ / [14]
            |========|
            |========|
   [11] \ \-|========|-\ \ [12]
                
                ^
   [13] \ \-|========|-\ \ [14]
            |========|
            |========|
   [11] / /-|========|-/ / [12]

'''


def SendSteeringCommand(device_id, steering_angle):
    #assert (steering_angle >= -20 and steering_angle <= 20)
    sendOutData = [0x69, 0x74, 0x72, 0x69, 0x00, 0x08, 0x01, 0x00, 0x02]
    distance = int((5 + steering_angle*0.25)*1000)
    distanceLo = (int)(distance%256)
    distanceHi = (int)(distance/256)
    sendOutData.append(distanceHi)
    sendOutData.append(distanceLo)
    sendOutData.append(device_id)
    control.SendDataAndGetResponse(sock=control.commandSock, data=sendOutData)

def CalWheelVal(gear, steeringWheel, thr, steeringType):# Return [m1s, m2s, m3s, m4s], [m1d, m2d, m3d, m4d], [m1a, m2a, m3a, m4a]
    steeringWheelAbs = abs(steeringWheel)
    refSpeed = ValueMapping(thr, 0, 255, 0, 500)
    motorPWMList = [0, 0, 0, 0]# Corresponds to motorIDList
    motorDirectionList = [0, 0, 0, 0]# Corresponds to motorIDList
    steeringAngList = [0, 0, 0, 0]# Corresponds to steeringIDList

    if (gear == 'Drive'):
        motorDirectionList = [2, 2, 2, 2]
    elif (gear == 'Reverse'):
        motorDirectionList = [1, 1, 1, 1]

    if (steeringWheelAbs < STEERINGWHEEL_TOLERANCE):
        return MotorRPMToPWM(refSpeed, refSpeed, refSpeed, refSpeed), motorDirectionList, steeringAngList

    innerAng = 0
    outerAng = 0

    innerVelo = 0
    outerVelo = 0

    if (steeringType == 1):# Ackermann
        steeringWheelAbsCorrection = GammaCorrection(steeringWheelAbs, 0.1, 0, 32768)
        steeringRadius = ValueMapping(steeringWheelAbsCorrection, 0, 32768, MAX_RADIUS_ACKERMANN, MIN_RADIUS_ACKERMANN)
        innerRadius = steeringRadius - CAR_WIDTH / 2
        outerRadius = steeringRadius + CAR_WIDTH / 2
        # Steering
        innerAng = math.degrees(math.atan2(CAR_LENGTH, innerRadius))
        outerAng = math.degrees(math.atan2(CAR_LENGTH, outerRadius))
        # Speed
        innerVelo = refSpeed * math.sqrt(innerRadius**2 + CAR_LENGTH**2) / steeringRadius
        outerVelo = refSpeed * math.sqrt(outerRadius**2 + CAR_LENGTH**2) / steeringRadius
        rearInnerVelo = refSpeed * innerRadius / steeringRadius
        rearOuterVelo = refSpeed * outerRadius / steeringRadius
        # Assign lists
        if (steeringWheel > 0):# Turn right, inner: 11
            if (gear == 'Drive'):
                steeringAngList = [innerAng, outerAng, 0, 0]
                motorPWMList = MotorRPMToPWM(innerVelo, outerVelo, rearInnerVelo, rearOuterVelo)
            elif (gear == 'Reverse'):
                steeringAngList = [0, 0, outerAng, innerAng]
                motorPWMList = MotorRPMToPWM(rearOuterVelo, rearInnerVelo, outerVelo, innerVelo)
        else:# Turn left
            innerAng = -innerAng
            outerAng = -outerAng
            if (gear == 'Drive'):
                steeringAngList = [outerAng, innerAng, 0, 0]
                motorPWMList = MotorRPMToPWM(outerVelo, innerVelo, rearOuterVelo, rearInnerVelo)
            elif (gear == 'Reverse'):
                steeringAngList = [0, 0, innerAng, outerAng]
                motorPWMList = MotorRPMToPWM(rearInnerVelo, rearOuterVelo, innerVelo, outerVelo)

    elif (steeringType == 2):# 4ws center baseline
        steeringWheelAbsCorrection = GammaCorrection(steeringWheelAbs, 0.1, 0, 32768)
        steeringRadius = ValueMapping(steeringWheelAbsCorrection, 0, 32768, MAX_RADIUS_4WS, MIN_RADIUS_4WS)
        innerRadius = steeringRadius - CAR_WIDTH / 2
        outerRadius = steeringRadius + CAR_WIDTH / 2
        # Steering
        innerAng = math.degrees(math.atan2(CAR_LENGTH / 2, innerRadius))
        outerAng = math.degrees(math.atan2(CAR_LENGTH / 2, outerRadius))
        # Speed
        innerVelo = refSpeed * math.sqrt(innerRadius**2 + (CAR_LENGTH / 2)**2) / steeringRadius
        outerVelo = refSpeed * math.sqrt(outerRadius**2 + (CAR_LENGTH / 2)**2) / steeringRadius
        # Assign lists
        if (steeringWheel > 0):# Turn right, inner: 11
            if (gear == 'Drive'):
                steeringAngList = [innerAng, outerAng, -innerAng, -outerAng]
                motorPWMList = MotorRPMToPWM(innerVelo, outerVelo, innerVelo, outerVelo)
            elif (gear == 'Reverse'):
                steeringAngList = [-outerAng, -innerAng, outerAng, innerAng]
                motorPWMList = MotorRPMToPWM(outerVelo, innerVelo, outerVelo, innerVelo)
        else:# Turn left
            innerAng = -innerAng
            outerAng = -outerAng
            if (gear == 'Drive'):
                steeringAngList = [outerAng, innerAng, -outerAng, -innerAng]
                motorPWMList = MotorRPMToPWM(outerVelo, innerVelo, outerVelo, innerVelo)
            elif (gear == 'Reverse'):
                steeringAngList = [-innerAng, -outerAng, innerAng, outerAng]
                motorPWMList = MotorRPMToPWM(innerVelo, outerVelo, innerVelo, outerVelo)
    
    steeringAngList[0] *= steeringCorrectionList[0]
    steeringAngList[1] *= steeringCorrectionList[1]
    steeringAngList[2] *= steeringCorrectionList[2]
    steeringAngList[3] *= steeringCorrectionList[3]
    return motorPWMList, motorDirectionList, steeringAngList


def ConvertSteeringWheelToCommand(gear : str, steeringWheel : int, thr : int, brk : int, clu : int, steeringType : int):
    motorPWMList = [0, 0, 0, 0]# Index from 0 to 3 corresponse to client ID from 11 to 14 respectively
    motorDirectionList = [0, 0, 0, 0]
    steeringAngList = [0, 0, 0, 0]
    # Process park gear and neutral gear
    if (gear == 'Park' or brk > 10):
        for _m_id in motorIDList:
            control.SendCommandSetAxleAndGetResponse(sock=control.commandSock, deviceId=_m_id, runDirection = 2, brake = 1, pwm = 0)
        return motorPWMList, steeringAngList, steeringType
    elif (gear == 'Neutral'):
        for _m_id in motorIDList:
            control.SendCommandSetAxleAndGetResponse(sock=control.commandSock, deviceId=_m_id, runDirection = 2, brake = 0, pwm = 0)
        return motorPWMList, steeringAngList, steeringType
    # Process drive gear and reverse gear
    if (steeringType != 3):# Ackermann: 1, 4ws: 2
        motorPWMList, motorDirectionList, steeringAngList = CalWheelVal(gear, steeringWheel, thr, steeringType)
    else:# Zero turn
        steeringAngList = [20, 20, -20, -20]
        if (steeringWheel > 0):
            motorDirectionList = [1, 2, 1, 2]
        elif (steeringWheel < 0):
            motorDirectionList = [2, 1, 2, 1]
        else:
            motorPWMList = [0, 0, 0, 0]
            motorDirectionList = [0, 0, 0, 0]
    
    retMotorList = []
    for _m_id, _m_dir, _m_val, _s_id, _s_val in zip(motorIDList, motorDirectionList, motorPWMList, steeringIDList, steeringAngList):
        control.SendCommandSetAxleAndGetResponse(sock=control.commandSock, deviceId=_m_id, runDirection=_m_dir, pwm=_m_val)
        SendSteeringCommand(_s_id, _s_val)

        # Process return value
        retMotorList.append(_m_val * -1 if (_m_dir == 2) else 1)
    
    return retMotorList, steeringAngList, steeringType


# New Method
###################################################################################################################################

from vehicle_interfaces.msg import Chassis
from vehicle_interfaces.msg import SteeringWheel

def CalSteeringWheelToChassis(swState : SteeringWheel):
    steeringWheel = swState.steering
    steeringWheelAbs = abs(swState.steering)
    refSpeed = ValueMapping(swState.pedal_throttle, 0, 255, 0, 120)
    motorPWMList = [0, 0, 0, 0]# Corresponds to motorIDList
    motorDirectionList = [0, 0, 0, 0]# Corresponds to motorIDList
    steeringAngList = [0, 0, 0, 0]# Corresponds to steeringIDList

    if (steeringWheelAbs < STEERINGWHEEL_TOLERANCE):
        steeringWheel = 0

    if (swState.func_0 == 3):
        if (steeringWheel > 0):
            motorDirectionList = [-1, 1, -1, 1]# 1->-1; 2->1
        elif (steeringWheel < 0):
            motorDirectionList = [1, -1, 1, -1]
    else:
        if (swState.gear == SteeringWheel.GEAR_DRIVE):
            motorDirectionList = [1, 1, 1, 1]
        elif (swState.gear == SteeringWheel.GEAR_REVERSE):
            motorDirectionList = [-1, -1, -1, -1]

    innerAng = 0
    outerAng = 0

    innerVelo = 0
    outerVelo = 0

    if (swState.func_0 == 3):# Zero turn
        steeringAngList = [-20, 20, 20, -20]
        motorPWMList = MotorRPMToPWM(refSpeed, refSpeed, refSpeed, refSpeed)

    elif (steeringWheel == 0):
        motorPWMList = MotorRPMToPWM(refSpeed, refSpeed, refSpeed, refSpeed)

    elif (swState.func_0 == 1):# Ackermann
        steeringWheelAbsCorrection = GammaCorrection(steeringWheelAbs, 0.2, 0, 32768)
        steeringRadius = ValueMapping(steeringWheelAbsCorrection, 0, 32768, MAX_RADIUS_ACKERMANN, MIN_RADIUS_ACKERMANN)
        innerRadius = steeringRadius - CAR_WIDTH / 2
        outerRadius = steeringRadius + CAR_WIDTH / 2
        # Steering
        innerAng = math.degrees(math.atan2(CAR_LENGTH, innerRadius))
        outerAng = math.degrees(math.atan2(CAR_LENGTH, outerRadius))
        # Speed
        innerVelo = refSpeed * math.sqrt(innerRadius**2 + CAR_LENGTH**2) / steeringRadius
        outerVelo = refSpeed * math.sqrt(outerRadius**2 + CAR_LENGTH**2) / steeringRadius
        rearInnerVelo = refSpeed * innerRadius / steeringRadius
        rearOuterVelo = refSpeed * outerRadius / steeringRadius
        # Assign lists
        if (steeringWheel > 0):# Turn right, inner: 11
            if (swState.gear == SteeringWheel.GEAR_DRIVE):
                steeringAngList = [innerAng, outerAng, 0, 0]
                motorPWMList = MotorRPMToPWM(innerVelo, outerVelo, rearInnerVelo, rearOuterVelo)
            elif (swState.gear == SteeringWheel.GEAR_REVERSE):
                steeringAngList = [0, 0, outerAng, innerAng]
                motorPWMList = MotorRPMToPWM(rearOuterVelo, rearInnerVelo, outerVelo, innerVelo)
        else:# Turn left
            innerAng = -innerAng
            outerAng = -outerAng
            if (swState.gear == SteeringWheel.GEAR_DRIVE):
                steeringAngList = [outerAng, innerAng, 0, 0]
                motorPWMList = MotorRPMToPWM(outerVelo, innerVelo, rearOuterVelo, rearInnerVelo)
            elif (swState.gear == SteeringWheel.GEAR_REVERSE):
                steeringAngList = [0, 0, innerAng, outerAng]
                motorPWMList = MotorRPMToPWM(rearInnerVelo, rearOuterVelo, innerVelo, outerVelo)

    elif (swState.func_0 == 2):# 4ws center baseline
        steeringWheelAbsCorrection = GammaCorrection(steeringWheelAbs, 0.2, 0, 32768)
        steeringRadius = ValueMapping(steeringWheelAbsCorrection, 0, 32768, MAX_RADIUS_4WS, MIN_RADIUS_4WS)
        innerRadius = steeringRadius - CAR_WIDTH / 2
        outerRadius = steeringRadius + CAR_WIDTH / 2
        # Steering
        innerAng = math.degrees(math.atan2(CAR_LENGTH / 2, innerRadius))
        outerAng = math.degrees(math.atan2(CAR_LENGTH / 2, outerRadius))
        # Speed
        innerVelo = refSpeed * math.sqrt(innerRadius**2 + (CAR_LENGTH / 2)**2) / steeringRadius
        outerVelo = refSpeed * math.sqrt(outerRadius**2 + (CAR_LENGTH / 2)**2) / steeringRadius
        # Assign lists
        if (steeringWheel > 0):# Turn right, inner: 11
            if (swState.gear == SteeringWheel.GEAR_DRIVE):
                steeringAngList = [innerAng, outerAng, -innerAng, -outerAng]
                motorPWMList = MotorRPMToPWM(innerVelo, outerVelo, innerVelo, outerVelo)
            elif (swState.gear == SteeringWheel.GEAR_REVERSE):
                steeringAngList = [-outerAng, -innerAng, outerAng, innerAng]
                motorPWMList = MotorRPMToPWM(outerVelo, innerVelo, outerVelo, innerVelo)
        else:# Turn left
            innerAng = -innerAng
            outerAng = -outerAng
            if (swState.gear == SteeringWheel.GEAR_DRIVE):
                steeringAngList = [outerAng, innerAng, -outerAng, -innerAng]
                motorPWMList = MotorRPMToPWM(outerVelo, innerVelo, outerVelo, innerVelo)
            elif (swState.gear == SteeringWheel.GEAR_REVERSE):
                steeringAngList = [-innerAng, -outerAng, innerAng, outerAng]
                motorPWMList = MotorRPMToPWM(innerVelo, outerVelo, innerVelo, outerVelo)

    elif (swState.func_0 == 4):# parallel
        steeringWheelAbsCorrection = GammaCorrection(steeringWheelAbs, 0.2, 0, 32768)
        steeringAngle = ValueMapping(steeringWheelAbsCorrection, 0, 32768, MIN_WHEEL_STEER_ANGLE, MAX_WHEEL_STEER_ANGLE)
        # Speed
        motorPWMList = MotorRPMToPWM(refSpeed, refSpeed, refSpeed, refSpeed)
        # Assign lists
        if (steeringWheel > 0):# Turn right, inner: 11
            steeringAngList = [steeringAngle, steeringAngle, steeringAngle, steeringAngle]
        else:# Turn left
            steeringAngList = [-steeringAngle, -steeringAngle, -steeringAngle, -steeringAngle]


    # for i in range(4):
    #     motorPWMList[i] *= motorDirectionList[i]
    #     steeringAngList[i] *= steeringCorrectionList[i]

    motorPWMList = [float(i) for i in motorPWMList]
    steeringAngList = [float(i) for i in steeringAngList]

    ret = Chassis()
    ret.wheel_num = 4
    ret.drive_motor = motorPWMList
    ret.steering_motor = steeringAngList
    ret.brake_motor = [0.0, 0.0, 0.0, 0.0]
    ret.parking_signal = [False, False, False, False]

    if (swState.gear == SteeringWheel.GEAR_PARK or swState.pedal_brake > 10):
        ret.drive_motor = [0.0, 0.0, 0.0, 0.0]
        ret.parking_signal = [True, True, True, True]
    elif (swState.gear == SteeringWheel.GEAR_NEUTRAL):
        ret.drive_motor = [0.0, 0.0, 0.0, 0.0]

    return ret
