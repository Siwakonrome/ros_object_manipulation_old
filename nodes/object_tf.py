#!/usr/bin/python3
import tf
import math
import time
import rospy
from ros_object_manipulation.Template import Position
from ros_object_manipulation.vision_tfhalper import VisionHalper
from ros_object_manipulation.Constant import StringSplit, ROSFramesId
from ros_object_manipulation.srv import GetCurrentObjectPosition, GetCurrentObjectPositionResponse, GetCurrentObjectPositionRequest


class ROSLink():
    ros_frames_id : ROSFramesId = ROSFramesId()
    camera_base_link: str = ros_frames_id.camera_base_link
    robot_ref_link = ros_frames_id.robot_ref_link 
    object_link: str = ros_frames_id.object_link   

class ROSService():
    camera_detect : str = "readPose_service"
    detect : str = "detect"
    base2object : str = "object_manipulation/object_goal_position"

class ROSParam():
    class Param():
        def __new__(self, name, default):
            return {"name":name, "default":default}

    def __init__(self, name):
        self.name = name
        self.current_tcp_config = '/{head_param}/current/tcp_config'.format(head_param=VisionHalper().header_of_param)
        self.is_broadcast_link = self.Param(f"{name}/broadcast_link", False)
        self.repeator_listen_tf = self.Param(f"{name}/repeator", 2000)
        self.camera_base_to_gnd_x = self.Param(f"{name}/base_x", 0.0)
        self.camera_base_to_gnd_y = self.Param(f"{name}/base_y", 0.0)
        self.camera_base_to_gnd_z = self.Param(f"{name}/base_z", 0.0)
        self.camera_base_to_gnd_rx = self.Param(f"{name}/base_roll", 0.0)
        self.camera_base_to_gnd_ry = self.Param(f"{name}/base_pitch", 0.0)
        self.camera_base_to_gnd_rz = self.Param(f"{name}/base_yaw", 0.0)
        self.rx_invited_min = self.Param(f"{name}/rx_invited_min", 0.0)
        self.rx_invited_max = self.Param(f"{name}/rx_invited_max", 50.0)

class ROSComponent():
    def init(self):
        try:
            rospy.init_node('object_tf_manipulation_node')
            self.node_name = rospy.get_name()
            self.params = ROSParam(self.node_name)
            self.vision_halper = VisionHalper()
            self.initilaize_services()
            self.initialize_boardcaster()
            self.initialize_params()
            self.initilaize_global_var()
        except Exception as e:
            rospy.logerr(format(e))
            rospy.logerr("object tf manipulation fail!!")
            return False
        return True

    def initialize_params(self):
        if not rospy.has_param(f'{self.params.is_broadcast_link["name"]}'):
            rospy.set_param(f'{self.params.is_broadcast_link["name"]}', self.params.is_broadcast_link["default"])
        if not rospy.has_param(f'{self.params.camera_base_to_gnd_x["name"]}'):
            rospy.set_param(f'{self.params.camera_base_to_gnd_x["name"]}', self.params.camera_base_to_gnd_x["default"])
        if not rospy.has_param(f'{self.params.camera_base_to_gnd_y["name"]}'):
            rospy.set_param(f'{self.params.camera_base_to_gnd_y["name"]}', self.params.camera_base_to_gnd_y["default"])
        if not rospy.has_param(f'{self.params.camera_base_to_gnd_z["name"]}'):
            rospy.set_param(f'{self.params.camera_base_to_gnd_z["name"]}', self.params.camera_base_to_gnd_z["default"])
        if not rospy.has_param(f'{self.params.camera_base_to_gnd_rx["name"]}'):
            rospy.set_param(f'{self.params.camera_base_to_gnd_rx["name"]}', self.params.camera_base_to_gnd_rx["default"])
        if not rospy.has_param(f'{self.params.camera_base_to_gnd_ry["name"]}'):
            rospy.set_param(f'{self.params.camera_base_to_gnd_ry["name"]}', self.params.camera_base_to_gnd_ry["default"])
        if not rospy.has_param(f'{self.params.camera_base_to_gnd_rz["name"]}'):
            rospy.set_param(f'{self.params.camera_base_to_gnd_rz["name"]}', self.params.camera_base_to_gnd_rz["default"])
        if not rospy.has_param(f'{self.params.repeator_listen_tf["name"]}'):
            rospy.set_param(f'{self.params.repeator_listen_tf["name"]}', self.params.repeator_listen_tf["default"])
        if not rospy.has_param(f'{self.params.rx_invited_min["name"]}'):
            rospy.set_param(f'{self.params.rx_invited_min["name"]}', self.params.rx_invited_min["default"])
        if not rospy.has_param(f'{self.params.rx_invited_max["name"]}'):
            rospy.set_param(f'{self.params.rx_invited_max["name"]}', self.params.rx_invited_max["default"])
    
    def initialize_boardcaster(self):
        self.base_cam_to_gnd_broadcaster_temp = tf.TransformBroadcaster()
        self.base_cam_to_gnd_broadcaster = tf.TransformBroadcaster()
        self.object_to_base_cam_broadcaster = tf.TransformBroadcaster()
    
    def initilaize_services(self):
        self.detect_servicer = rospy.Service(ROSService.detect, GetCurrentObjectPosition, self.detect_callback)
        self.camera_detect_servicer = rospy.ServiceProxy(ROSService.camera_detect, GetCurrentObjectPosition)
        self.get_transform_base2object = rospy.ServiceProxy(ROSService.base2object, GetCurrentObjectPosition)

    def initilaize_global_var(self):
        global object_pose, object_time_stemp
        object_pose = None
        object_time_stemp = str(rospy.Time.now())
    
    def get_param(self, param:ROSParam):
        try:
            return rospy.get_param(param['name'])
        except Exception:
            return param['default']

    def check_current_tcp(self):
        ret_list, ret = [], False
        tcp_data = [f'{self.params.current_tcp_config}/x'
                   ,f'{self.params.current_tcp_config}/y'
                   ,f'{self.params.current_tcp_config}/z'
                   ,f'{self.params.current_tcp_config}/rx'
                   ,f'{self.params.current_tcp_config}/ry'
                   ,f'{self.params.current_tcp_config}/rz'
                   ,f'{self.params.current_tcp_config}/name']
        for item in tcp_data:
            ret_list.append(rospy.has_param(item))
        if all(ret_list) is True:
            ret = True
        return ret, tcp_data

    def get_camera_position(self):
        position = Position()
        name : str = str()
        try:
            ret, tcp_data = self.check_current_tcp()
            if ret:
                name = rospy.get_param(tcp_data[6])
                position.pose.set(rospy.get_param(tcp_data[0]),rospy.get_param(tcp_data[1]),rospy.get_param(tcp_data[2]))
                position.orientation.set(rospy.get_param(tcp_data[3]), rospy.get_param(tcp_data[4]), rospy.get_param(tcp_data[5]))
            else:
                position.pose.set(rospy.get_param(f'{self.params.camera_base_to_gnd_x["name"]}'),rospy.get_param(f'{self.params.camera_base_to_gnd_y["name"]}'),rospy.get_param(f'{self.params.camera_base_to_gnd_z["name"]}'))
                position.orientation.set(rospy.get_param(f'{self.params.camera_base_to_gnd_rx["name"]}'), rospy.get_param(f'{self.params.camera_base_to_gnd_ry["name"]}'), rospy.get_param(f'{self.params.camera_base_to_gnd_rz["name"]}'))
        except Exception as e:
            rospy.logerr(format(e))
        return position, name

    def broadcast_camera_base_to_ref_link(self, *args, **kwargs):
        if self.get_param(self.params.is_broadcast_link):
            now_time = rospy.Time.now()
            cam_position, detect_name = self.get_camera_position()
            self.base_cam_to_gnd_broadcaster.sendTransform(cam_position.pose(),
                                                            cam_position.orientation(),
                                                            now_time,
                                                            ROSLink.camera_base_link,
                                                            ROSLink.robot_ref_link)
            time.sleep(0.2)
            now_time__temp = rospy.Time.now()
            self.base_cam_to_gnd_broadcaster_temp.sendTransform(cam_position.pose(),
                                                            cam_position.orientation(),
                                                            now_time__temp,
                                                            ROSLink.camera_base_link,
                                                            "{rrl}_{name}".format(rrl=ROSLink.robot_ref_link, name=detect_name))
    
    def broadcast_object_to_base_link(self, object_position:Position, object_time_stemp : str, *args, **kwargs):
        if self.get_param(self.params.is_broadcast_link):
            now_time = rospy.Time.now()
            self.object_to_base_cam_broadcaster.sendTransform(object_position.pose(),
                                                            object_position.orientation(),
                                                            now_time,
                                                            "{ob}_{time}".format(ob=ROSLink.object_link, time=object_time_stemp),
                                                            ROSLink.camera_base_link)
    
    def detect_callback(self, req):
        raise NotImplementedError

def deg_to_rad(deg):
    return deg * math.pi / 180.0

def take_repeator_listen_tf(this_node : ROSComponent, object_time_stemp : str) -> GetCurrentObjectPositionResponse:
    object_frame_time_stemp : str = "{ob}_{time}".format(ob=ROSLink.object_link, time=object_time_stemp)
    this_node.vision_halper.wait_tf_canTransform(ROSLink.camera_base_link, object_frame_time_stemp)
    result : GetCurrentObjectPositionResponse = this_node.get_transform_base2object(object_frame_time_stemp.replace(StringSplit.SL, str())) 
    return result

def if_object_to_base_not_invited(this_node : ROSComponent, pose : GetCurrentObjectPositionResponse) -> GetCurrentObjectPositionResponse:
    rx_abs : float = float(abs(pose.rx))
    rx_invited_min : float = rospy.get_param(f'{this_node.params.rx_invited_min["name"]}')
    rx_invited_max : float = rospy.get_param(f'{this_node.params.rx_invited_max["name"]}')
    if (rx_abs >= rx_invited_min) and (rx_abs <= rx_invited_max): pose.ack = False
    return pose

def detect_callback(node : ROSComponent, req : GetCurrentObjectPositionRequest):
    global object_pose, object_time_stemp
    resp : GetCurrentObjectPositionResponse = node.camera_detect_servicer(req.name)
    object_pose = Position()
    posi_x, posi_y, posi_z = resp.x / 1000.0, resp.y / 1000.0, resp.z / 1000.0
    rpy_x, rpy_y, rpy_z = deg_to_rad(resp.rx), deg_to_rad(resp.ry), deg_to_rad(resp.rz)
    object_pose.pose.set(x = posi_x, y = posi_y, z = posi_z)
    object_pose.orientation.set(*[rpy_x, rpy_y, rpy_z])
    object_time_stemp = str(rospy.Time.now())
    result : GetCurrentObjectPositionResponse = take_repeator_listen_tf(this_node=node, object_time_stemp=object_time_stemp)
    result.ack = resp.ack
    return if_object_to_base_not_invited(this_node=node,pose=result)

def main():
    rospy.loginfo("Initializing object tf manipulation")
    node = ROSComponent()
    node.detect_callback = lambda req : detect_callback(node, req)
    if not node.init():
        rospy.logerr("object tf manipulation node error, program terminate!!")
        return
    else:
        rospy.loginfo("object tf manipulation node initialize completed!!")
    rospy.loginfo("Object tf manipulation begin!!")
    def loop(event):
        global object_pose, object_time_stemp
        if object_pose is not None:
            node.broadcast_object_to_base_link(object_position=object_pose, object_time_stemp=object_time_stemp)
    rospy.Timer(rospy.Duration(1.0/1000.0), node.broadcast_camera_base_to_ref_link)
    rospy.Timer(rospy.Duration(1.0/10.0), loop)
    rospy.spin()

if __name__ == "__main__":
    main()


    