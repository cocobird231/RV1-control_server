import sys
sys.path.insert(0, '/lib')
import rclpy
from ros2_utils import ControlParameters
import entrypoint_idclient
import entrypoint_ros2_sub

def main(args=None):
    ######## ROS2 ########
    rclpy.init(args=args)
    params = ControlParameters('controlserver_params_node')
    print('Operation mode:', params.operationMode)
    print('Pub/Sub topic:', params.topicName)
    if (params.operationMode == 'IDClient'):
        entrypoint_idclient.main(params)
    elif (params.operationMode == 'ROS2_sub'):
        entrypoint_ros2_sub.main(params)
    print('Process Ended.')