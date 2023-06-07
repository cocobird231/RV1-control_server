import pyIDClient
import threading

################################################################################################
#                                       ID Client Definitions
################################################################################################

# Change those parameters may not work with some ID server
PACKET_HEADER_SIZE = 22
PACKET_PAYLOAD_SIZE = 1450

# Controller remote flag
remoteDevName = ''
isRemoteF = False
idcLock = threading.Lock()

# This function will be called every client message received
def func(idc, deviceName, msg):
    global isRemoteF
    global remoteDevName
    global idcLock
    #print("<RecvMsgEventHandler> [%s]: %s" %(deviceName, msg))
    if (msg == 'RemoteRegister' and not isRemoteF):
        try:
            idc.sendMsgToClient(deviceName, 'ControlRegister')
            idcLock.acquire()
            remoteDevName = deviceName
            isRemoteF = True
            idcLock.release()
        except Exception as e:
            print('!!!RecvMsgEventHandler Exception!!!', idc.getIDTable(), e)
            idc.requestIDTableFromServer()
            