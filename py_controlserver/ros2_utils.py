import rclpy
from rclpy.node import Node
from vehicle_interfaces.msg import WheelState

import threading

################################################################################################
#                                       ROS2 Definitions
################################################################################################
class ControlParameters(Node):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.operationMode = 'IDClient'
        self.sendInterval = 0.2

        self.internalIDServerIP = '192.168.1.42'
        self.internalIDServerPort = 0
        self.internalIDServerDeviceID = 0

        self.externalIDServerIP = '61.220.23.240'
        self.externalIDServerPort = '10000'
        self.externalIDServerDeviceID = 'CAR1'
        
        self.topicName = 'remotecomm_0'
        self.publishInterval = 0.2
        self.gndDetectNode = 'grounddetect_0_node'

        self.nodeName = 'controlserver'
        self.id = 0
        self.qosService = 'qos_0'
        self.safetyService = 'safety_0'
        self.timesyncService = 'timesync_0'
        
        self.declare_parameter('operationMode', self.operationMode)
        self.declare_parameter('sendInterval', self.sendInterval)

        self.declare_parameter('internalIDServerIP', self.internalIDServerIP)
        self.declare_parameter('internalIDServerPort', self.internalIDServerPort)
        self.declare_parameter('internalIDServerDeviceID', self.internalIDServerDeviceID)

        self.declare_parameter('externalIDServerIP', self.externalIDServerIP)
        self.declare_parameter('externalIDServerPort', self.externalIDServerPort)
        self.declare_parameter('externalIDServerDeviceID', self.externalIDServerDeviceID)
        
        self.declare_parameter('topicName', self.topicName)
        self.declare_parameter('publishInterval', self.publishInterval)
        self.declare_parameter('gndDetectNode', self.gndDetectNode)

        self.declare_parameter('nodeName', self.nodeName)
        self.declare_parameter('id', self.id)
        self.declare_parameter('qosService', self.qosService)
        self.declare_parameter('safetyService', self.safetyService)
        self.declare_parameter('timesyncService', self.timesyncService)
        self._getParam()
    
    def _getParam(self):
        self.operationMode = rclpy.parameter.parameter_value_to_python(self.get_parameter('operationMode').get_parameter_value())
        self.sendInterval = rclpy.parameter.parameter_value_to_python(self.get_parameter('sendInterval').get_parameter_value())

        self.internalIDServerIP = rclpy.parameter.parameter_value_to_python(self.get_parameter('internalIDServerIP').get_parameter_value())
        self.internalIDServerPort = rclpy.parameter.parameter_value_to_python(self.get_parameter('internalIDServerPort').get_parameter_value())
        self.internalIDServerDeviceID = rclpy.parameter.parameter_value_to_python(self.get_parameter('internalIDServerDeviceID').get_parameter_value())


        self.externalIDServerIP = rclpy.parameter.parameter_value_to_python(self.get_parameter('externalIDServerIP').get_parameter_value())
        self.externalIDServerPort = rclpy.parameter.parameter_value_to_python(self.get_parameter('externalIDServerPort').get_parameter_value())
        self.externalIDServerDeviceID = rclpy.parameter.parameter_value_to_python(self.get_parameter('externalIDServerDeviceID').get_parameter_value())

        self.topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topicName').get_parameter_value())
        self.publishInterval = rclpy.parameter.parameter_value_to_python(self.get_parameter('publishInterval').get_parameter_value())
        self.gndDetectNode = rclpy.parameter.parameter_value_to_python(self.get_parameter('gndDetectNode').get_parameter_value())

        self.nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('nodeName').get_parameter_value())
        self.id = rclpy.parameter.parameter_value_to_python(self.get_parameter('id').get_parameter_value())
        self.qosService = rclpy.parameter.parameter_value_to_python(self.get_parameter('qosService').get_parameter_value())
        self.safetyService = rclpy.parameter.parameter_value_to_python(self.get_parameter('safetyService').get_parameter_value())
        self.timesyncService = rclpy.parameter.parameter_value_to_python(self.get_parameter('timesyncService').get_parameter_value())
        self.nodeName += '_' + str(self.id) + '_node'

class WheelStatePublisher(Node):
    def __init__(self, nodeName : str, topicName : str, interval_s : float):
        super().__init__(nodeName)
        self.nodeName_ = nodeName
        self.publisher_ = self.create_publisher(WheelState, topicName, 10)
        timer_period = interval_s  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = WheelState()
        self.msg.header.priority = self.msg.header.PRIORITY_CONTROL
        self.msg.header.device_type = self.msg.header.DEVTYPE_STEERINGWHEEL
        self.msg.header.device_id = nodeName
        self.msgLock = threading.Lock()
    
    def setMsg(self, gear : str, steering : int, thr : int, brk : int, clu : int, button : int, func : int):
        self.msgLock.acquire()
        self.msg.header.stamp = self.get_clock().now().to_msg()
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
        sendMsg = self.msg
        self.msgLock.release()

        self.publisher_.publish(sendMsg)
        # self.get_logger().info('Pub: %03d|%05d|%05d|%05d|%05d|%03d|%03d' %(sendMsg.gear, sendMsg.steering, \
        #     sendMsg.pedal_throttle, sendMsg.pedal_brake, sendMsg.pedal_clutch, sendMsg.button, sendMsg.func))

class WheelStateSubscriber(Node):
    def __init__(self, nodeName, topicName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(WheelState, topicName, self.listener_callback, 10)
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

