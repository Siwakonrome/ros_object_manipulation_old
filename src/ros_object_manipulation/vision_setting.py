import json
import math
import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations as frames_methods
from std_msgs.msg import String
from .CustomStruct import VisionSettingTf, VisionSettingParam, Offset
from .Constant import POffSet, TCPOffSet, StringSplit, NachiStructList, NachiConfigPosition



class VisionSetting:
    def __init__(self) -> None:
        self.ros_base_link : str = "base_link"
        self.param : str = "param_visionsetting"
        self.dotnetdatabase : str = "dotnetdatabase"
        self.tosub_topic : str = "configuration_enable"
        self.poffset : POffSet = POffSet()   
        self.tcpoffset : TCPOffSet = TCPOffSet()
        self.br = tf2_ros.TransformBroadcaster()
        self.string_split : StringSplit = StringSplit()
        self.nachi_structs : NachiStructList = NachiStructList() 
        self.tf_name : NachiConfigPosition = NachiConfigPosition() 
        
    def set_vision_setting_tf(self, config_tf) -> VisionSettingTf:
        set_setting : VisionSettingTf = VisionSettingTf()
        set_setting.model_name = config_tf['name']
        set_setting.c_t = config_tf['pos_config_c_t']
        set_setting.c_agv = config_tf['pos_config_c_agv']
        set_setting.p_t = config_tf['pos_config_p_t']
        set_setting.p_agv = config_tf['pos_config_p_agv']
        return set_setting

    def set_vision_setting_param(self, config_param) -> VisionSettingParam:
        set_setting : VisionSettingParam = VisionSettingParam()
        set_setting.model_name = config_param['name']
        set_setting.score = config_param['score']
        set_setting.gripper_id = config_param['gripper_id']
        set_setting.gripper_pos = config_param['gripper_pos']
        set_setting.camera_params = config_param['camera_params']
        return set_setting

    def set_offset_arm(self, offset_string) -> Offset:
        offset : Offset = Offset()
        pose = offset_string.split(self.string_split.SL)
        if (offset_string == self.string_split.EMPTY):
            offset.x, offset.y, offset.z, offset.rx, offset.ry, offset.rz = 0.0,0.0,0.0,0.0,0.0,0.0
        else:
            offset.x, offset.y, offset.z, offset.rx, offset.ry, offset.rz = float(pose[0]),float(pose[1]),float(pose[2]),float(pose[3]),float(pose[4]),float(pose[5])
        return offset
    
    def configuration_table_callback(self, data:String) -> None:
        self.vision_setting = data.data 
        for item in json.loads(self.vision_setting):
            self.this_config = item
            self.set_tf_setting : VisionSettingTf = self.set_vision_setting_tf(self.this_config)
            self.share_vision_setting_tfs(self.set_tf_setting.p_t, self.tf_name.P_T , self.set_tf_setting.model_name)
            self.share_vision_setting_tfs(self.set_tf_setting.p_agv, self.tf_name.P_AGV, self.set_tf_setting.model_name)
            self.share_vision_setting_tf(self.set_tf_setting.c_t, self.tf_name.C_T, self.set_tf_setting.model_name)
            self.share_vision_setting_tf(self.set_tf_setting.c_agv, self.tf_name.C_AGV, self.set_tf_setting.model_name)
            self.share_vision_setting_param(self.this_config)

    def share_vision_setting_param(self, this_config) -> None:
        self.set_param_setting : VisionSettingParam = self.set_vision_setting_param(this_config)
        self.p_offset : Offset = self.set_offset_arm(this_config['p_offset'])
        self.tcp_config : Offset = self.set_offset_arm(this_config['tcp_config'])  
        self.score = self.string_split.EMPTY if (self.set_param_setting.score == self.string_split.EMPTY) else float(self.set_param_setting.score)
        self.gripper_id = self.string_split.EMPTY if (self.set_param_setting.gripper_id == self.string_split.EMPTY) else int(self.set_param_setting.gripper_id)
        self.gripper_pos = self.string_split.EMPTY if (self.set_param_setting.gripper_pos == self.string_split.EMPTY) else int(self.set_param_setting.gripper_pos)
        self.camera_params = self.string_split.EMPTY if (self.set_param_setting.camera_params == self.string_split.EMPTY) else self.set_param_setting.camera_params   
        rospy.set_param("/{head}/{name}/score".format(head=self.param, name=self.set_param_setting.model_name), self.score)
        rospy.set_param("/{head}/{name}/gripper/id".format(head=self.param, name=self.set_param_setting.model_name), self.gripper_id)
        rospy.set_param("/{head}/{name}/gripper/pos".format(head=self.param, name=self.set_param_setting.model_name), self.gripper_pos)
        rospy.set_param("/{head}/{name}/camera_params".format(head=self.param, name=self.set_param_setting.model_name), self.camera_params)  
        for item in self.nachi_structs.struct_list: 
            rospy.set_param("/{head}/{name}/{op}".format(head=self.param, name=self.set_param_setting.model_name, op=getattr(self.poffset, item)), getattr(self.p_offset, item))
            rospy.set_param("/{head}/{name}/gripper/{tcp}".format(head=self.param, name=self.set_param_setting.model_name, tcp=getattr(self.tcpoffset, item)), getattr(self.tcp_config, item))

    def share_vision_setting_tf(self, this_pos : str, this_pos_name : str, model_name : str) -> None:
        if (this_pos != self.string_split.EMPTY):
            pos_ypr = this_pos.split(self.string_split.SL)
            quaternion = frames_methods.quaternion_from_euler(float(pos_ypr[5]) * (math.pi/180), float(pos_ypr[4]) * (math.pi/180), float(pos_ypr[3]) * (math.pi/180))
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.ros_base_link
            t.child_frame_id = "{model_name}/{this_pos_name}".format(model_name=model_name, this_pos_name=this_pos_name)
            t.transform.translation.x = float(pos_ypr[0]) / 1000
            t.transform.translation.y = float(pos_ypr[1]) / 1000
            t.transform.translation.z = float(pos_ypr[2]) / 1000
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]
            self.br.sendTransform(t)

    def share_vision_setting_tfs(self, this_pos : str, this_pos_name : str, model_name : str) -> None:
        if (this_pos != self.string_split.EMPTY):
            num = 0
            for item in this_pos.split(self.string_split.CM):
                pos_ypr = item.split(self.string_split.SL)
                quaternion = frames_methods.quaternion_from_euler(float(pos_ypr[5]) * (math.pi/180), float(pos_ypr[4]) * (math.pi/180), float(pos_ypr[3]) * (math.pi/180))
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.ros_base_link
                t.child_frame_id = "{model_name}/{this_pos_name}{num}".format(model_name=model_name, this_pos_name=this_pos_name,num=num)
                t.transform.translation.x = float(pos_ypr[0]) / 1000
                t.transform.translation.y = float(pos_ypr[1]) / 1000
                t.transform.translation.z = float(pos_ypr[2]) / 1000
                t.transform.rotation.x = quaternion[0]
                t.transform.rotation.y = quaternion[1]
                t.transform.rotation.z = quaternion[2]
                t.transform.rotation.w = quaternion[3]
                self.br.sendTransform(t)
                num += 1

def share_vision_setting() -> None:
    visionSetting = VisionSetting()
    dotnetdatabase : str = visionSetting.dotnetdatabase
    topics : str = visionSetting.tosub_topic 
    rospy.Subscriber('/{node}/{topic}'.format(node=dotnetdatabase, topic=topics), String, callback = visionSetting.configuration_table_callback)
    rospy.spin()

def main() -> None:
    share_vision_setting()

