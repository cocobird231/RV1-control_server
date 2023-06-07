from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml
from yaml import load, Loader

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory('py_controlserver'), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)
    return LaunchDescription([
        Node(
            package="py_controlserver",
            namespace=data['node_prop']['namespace'],
            executable="run",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "operationMode" : data['control_prop']['operationMode'], 
                    "sendInterval" : data['control_prop']['sendInterval'], 
                    "internalIDServerIP" : data['internal_prop']['hostIP'], 
                    "internalIDServerPort" : data['internal_prop']['port'], 
                    "internalIDServerDeviceID" : data['internal_prop']['ID'], 
                    "externalIDServerIP" : data['external_prop']['hostIP'], 
                    "externalIDServerPort" : data['external_prop']['port'], 
                    "externalIDServerDeviceID" : data['external_prop']['ID'], 
                    "topicName" : data['topic_Control']['topicName'], 
                    "publishInterval" : data['topic_Control']['publishInterval'], 
                    "serviceName" : data['service_SafetyServer']['serviceName'], 
                    "gndDetectNode" : data['service_SafetyServer']['gndDetectNode'], 
                    "mainNodeName" : data['node_prop']['nodeName'], 
                }
            ]
        )
    ])
