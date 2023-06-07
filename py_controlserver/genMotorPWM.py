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
#                                       SteeringWheel Parameters
################################################################################################
STEERINGWHEEL_TOLERANCE = 1500# left < -1500, 1500 > right
CAR_LENGTH = 2200# unit: mm
CAR_WIDTH = 2080# unit: mm
MAX_RADIUS_ACKERMANN = 60000# unit: mm
MIN_RADIUS_ACKERMANN = 7100# unit: mm
MAX_RADIUS_4WS = 30000# unit: mm
MIN_RADIUS_4WS = 4100# unit: mm



################################################################################################
#                                       Motor Parameter Definitions
################################################################################################
limitRPM = 500
limitPWM = 40
motorIDList = [11, 12, 13, 14]
steeringIDList = [41, 42, 43, 44]

m1_y = np.array([10, 15, 20, 25, 30, 40, 50])
m2_y = np.array([10, 15, 20, 25, 30, 40, 50])
m3_y = np.array([10, 15, 20, 25, 30])
m4_y = np.array([10, 15, 20, 25, 30, 40, 50])

m1_x = np.array([121.3, 211.4, 272.6, 316.5, 352.8, 403.3, 444.5])
m2_x = np.array([91.6, 197.5, 261.6, 308.1, 344, 400, 436.2])
m3_x = np.array([78, 187.1, 255.2, 302.4, 324.7])
m4_x = np.array([105.7, 201.1, 262.2, 309.1, 345.1, 398.2, 440.8])

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
