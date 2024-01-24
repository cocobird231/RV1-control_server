import rclpy
from rclpy.node import Node
from vehicle_interfaces.msg import Header
from vehicle_interfaces.msg import WheelState
from vehicle_interfaces.params import GenericParams
from vehicle_interfaces.vehicle_interfaces import VehicleServiceNode

import threading

################################################################################################
#                                       ROS2 Definitions
################################################################################################
class ControlParameters(GenericParams):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.operationMode = 'IDClient'
        self.sendInterval_s = 0.2

        self.internalIDServerIP = '192.168.1.42'
        self.internalIDServerPort = 0
        self.internalIDServerDeviceID = 0

        self.externalIDServerIP = '61.220.23.240'
        self.externalIDServerPort = '10000'
        self.externalIDServerDeviceID = 'CAR1'

        self.serviceName = 'controlserver_0'
        
        self.topicName = 'remotecomm_0'
        self.publishInterval_s = 0.2
        self.externalTimeout_ms = 2000.0


        self.declare_parameter('operationMode', self.operationMode)
        self.declare_parameter('sendInterval_s', self.sendInterval_s)

        self.declare_parameter('internalIDServerIP', self.internalIDServerIP)
        self.declare_parameter('internalIDServerPort', self.internalIDServerPort)
        self.declare_parameter('internalIDServerDeviceID', self.internalIDServerDeviceID)

        self.declare_parameter('externalIDServerIP', self.externalIDServerIP)
        self.declare_parameter('externalIDServerPort', self.externalIDServerPort)
        self.declare_parameter('externalIDServerDeviceID', self.externalIDServerDeviceID)

        self.declare_parameter('serviceName', self.serviceName)
        
        self.declare_parameter('topicName', self.topicName)
        self.declare_parameter('publishInterval_s', self.publishInterval_s)
        self.declare_parameter('externalTimeout_ms', self.externalTimeout_ms)
        self._getParam()
    
    def _getParam(self):
        self.operationMode = self.get_parameter('operationMode').get_parameter_value().string_value
        self.sendInterval_s = self.get_parameter('sendInterval_s').get_parameter_value().double_value

        self.internalIDServerIP = self.get_parameter('internalIDServerIP').get_parameter_value().string_value
        self.internalIDServerPort = self.get_parameter('internalIDServerPort').get_parameter_value().integer_value
        self.internalIDServerDeviceID = self.get_parameter('internalIDServerDeviceID').get_parameter_value().integer_value

        self.externalIDServerIP = self.get_parameter('externalIDServerIP').get_parameter_value().string_value
        self.externalIDServerPort = self.get_parameter('externalIDServerPort').get_parameter_value().string_value
        self.externalIDServerDeviceID = self.get_parameter('externalIDServerDeviceID').get_parameter_value().string_value

        self.serviceName = self.get_parameter('serviceName').get_parameter_value().string_value

        self.topicName = self.get_parameter('topicName').get_parameter_value().string_value
        self.publishInterval_s = self.get_parameter('publishInterval_s').get_parameter_value().double_value
        self.externalTimeout_ms = self.get_parameter('externalTimeout_ms').get_parameter_value().double_value

class WheelStatePublisher(VehicleServiceNode):
    def __init__(self, params : ControlParameters):
        super().__init__(params)
        self.publisher_ = self.create_publisher(WheelState, params.topicName, 10)
        self.timer = self.create_timer(params.publishInterval_s, self.timer_callback)
        self.frame_id_ = 0
        self.msg = WheelState()
        self.msg.header.priority = Header.PRIORITY_CONTROL
        self.msg.header.device_type = Header.DEVTYPE_STEERINGWHEEL
        self.msg.header.device_id = params.nodeName
        self.msg.header.ref_publish_time_ms = params.publishInterval_s * 1000.0
        self.msgLock = threading.Lock()
    
    def setMsg(self, gear : str, steering : int, thr : int, brk : int, clu : int, button : int, func : int):
        self.msgLock.acquire()
        if (gear == 'Park'):
            self.msg.gear = self.msg.GEAR_PARK
        elif (gear == 'Reverse'):
            self.msg.gear = self.msg.GEAR_REVERSE
        elif (gear == 'Neutral'):
            self.msg.gear = self.msg.GEAR_NEUTRAL
        elif (gear == 'Drive'):
            self.msg.gear = self.msg.GEAR_DRIVE
        self.msg.steering = steering
        self.msg.pedal_throttle = thr
        self.msg.pedal_brake = brk
        self.msg.pedal_clutch = clu
        self.msg.button = button
        self.msg.func = func
        self.msgLock.release()

    def timer_callback(self):
        self.msgLock.acquire()
        self.msg.header.frame_id = self.frame_id_
        self.frame_id_ += 1
        self.msg.header.stamp_type = self.getTimestampType()
        self.msg.header.stamp = self.getTimestamp().to_msg()
        self.msg.header.stamp_offset = self.getCorrectDuration().nanoseconds
        sendMsg = self.msg
        self.msgLock.release()

        self.publisher_.publish(sendMsg)
        # self.get_logger().info('Pub: %03d|%05d|%05d|%05d|%05d|%03d|%03d' %(sendMsg.gear, sendMsg.steering, \
        #     sendMsg.pedal_throttle, sendMsg.pedal_brake, sendMsg.pedal_clutch, sendMsg.button, sendMsg.func))

class WheelStateSubscriber(VehicleServiceNode):
    def __init__(self, params : ControlParameters):
        super().__init__(params.nodeName)
        self.subscription = self.create_subscription(WheelState, params.topicName, self.listener_callback, 10)
        self.gear_ = 'Park'
        self.msg_ = WheelState()
        self.msgLock = threading.Lock()

    def listener_callback(self, msg):
        self.msgLock.acquire()
        self.msg_ = msg
        if (msg.gear == msg.GEAR_PARK):
            self.gear_ = 'Park'
        elif (msg.gear == msg.GEAR_REVERSE):
            self.gear_ = 'Reverse'
        elif (msg.gear == msg.GEAR_NEUTRAL):
            self.gear_ = 'Neutral'
        elif (msg.gear == msg.GEAR_DRIVE):
            self.gear_ = 'Drive'
        self.msgLock.release()

        self.get_logger().info('I heard: %03d|%05d|%05d|%05d|%05d|%03d|%03d' %(msg.gear, msg.steering, \
            msg.pedal_throttle, msg.pedal_brake, msg.pedal_clutch, msg.button, msg.func))
        # ConvertSteeringWheelToCommand(gear, msg.steering, msg.pedal_throttle, msg.pedal_brake, msg.pedal_clutch, msg.button)
    
    def getMsg(self):
        self.msgLock.acquire()
        ret0 = self.gear_
        ret = self.msg_
        self.msgLock.release()
        return ret0, ret.steering, ret.pedal_throttle, ret.pedal_brake, ret.pedal_clutch, ret.button

