from ros2_utils import ControlParameters, WheelStateSubscriber
from control import *
# TODO: Update ros2_sub method. Update to support safety service.
def main(params):
    ######## ROS2 ########
    wsSub = WheelStateSubscriber(params.nodeName, params.topicName)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(wsSub)
    executorTH = threading.Thread(target=executor.spin, daemon=True)
    executorTH.start()


    ######## Control ########
    global commandSock
    
    connF = False
    while (not connF):
        try:
            dataSock = sockConnect(params.internalIDServerIP)
            sendCommandTellId(dataSock, params.internalIDServerDeviceID)
            connF = True
        except:
            print('Retry dataSock connection...')
            time.sleep(1)
    
    connF = False
    while (not connF):
        try:
            imAliveSock = sockConnectImAlive(params.internalIDServerIP)    
            receivedData = sendCommandTellId(imAliveSock, params.internalIDServerDeviceID)
            connF = True
        except:
            print('Retry imAliveSock connection...')
            time.sleep(1)

    handler1 = Thread(target=LoopSendAlive, args=(imAliveSock, ))
    handler1.start()
    
    connF = False
    while (not connF):
        try:
            commandSock = sockConnectCommand(params.internalIDServerIP)
            connF = True
        except:
            print('Retry commandSock connection...')
            time.sleep(1)
    #handler2 = Thread(target=LoopHandleJoystickEvents, args=(commandSock, ))
    #handler2.start()
    handler1.join()
    executorTH.join()
