#!/usr/bin/python3
import json
import rospy
import socket
from ros_object_manipulation.srv import GetCurrentObjectPosition, GetCurrentObjectPositionResponse, GetCurrentObjectPositionRequest



class ROSParam():
    class Param():
        def __new__(self, name, default):
            return {"name":name, "default":default}
    def __init__(self, name):
        self.name = name
        self.dotnet_communication_ip = self.Param(f"{name}/ip", "127.0.0.1")
        self.dotnet_communication_port = self.Param(f"{name}/port", 8800)

class TriggerHalconDotNet:
    def __init__(self):
        rospy.init_node('readPose_node')
        self.node_name = rospy.get_name()
        self.params = ROSParam(self.node_name)
        self.initialize_params()
        self.HOST = rospy.get_param(f'{self.params.dotnet_communication_ip["name"]}')
        self.PORT = rospy.get_param(f'{self.params.dotnet_communication_port["name"]}')
        self.encoding = 'utf-8'
        self.service = rospy.Service('readPose_service', GetCurrentObjectPosition, self.callback_readPose)

    def initialize_params(self):
        if not rospy.has_param(f'{self.params.dotnet_communication_ip["name"]}'):
            rospy.set_param(f'{self.params.dotnet_communication_ip["name"]}', self.params.dotnet_communication_ip["default"])
        if not rospy.has_param(f'{self.params.dotnet_communication_port["name"]}'):
            rospy.set_param(f'{self.params.dotnet_communication_port["name"]}', self.params.dotnet_communication_port["default"])

    def callback_readPose(self, request : GetCurrentObjectPositionRequest):
        object_pose : GetCurrentObjectPositionResponse = GetCurrentObjectPositionResponse()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            reqText = request.name
            s.connect((self.HOST, self.PORT))
            s.sendall(reqText.encode())
            data = s.recv(1024).decode(self.encoding)
        data_dict = json.loads(data)
        object_pose.x = data_dict['x']
        object_pose.y = data_dict['y']
        object_pose.z = data_dict['z']
        object_pose.rx = data_dict['rx']
        object_pose.ry = data_dict['ry']
        object_pose.rz = data_dict['rz']
        if data_dict['modelName'] == str():
            object_pose.ack = False
        else:
            object_pose.ack = True
        s.close()
        return object_pose
  
def main():
    TriggerHalconDotNet()
    rospy.spin()
    
    
if __name__ == "__main__":
    main()









