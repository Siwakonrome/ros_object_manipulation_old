import tf
import math
import rospy
import random
import tf2_ros
from tf import TransformListener
from .CustomStruct import EulerFromQuaternionStruct
from geometry_msgs.msg import TransformStamped, Quaternion
from ros_object_manipulation.srv import GetCurrentObjectPositionResponse
from .Constant import POffSet, NachiConfigPosition, NachiStructList, ROSFramesId, StringSplit

pt_memory = []
pagv_memory = []

class VisionHalper:
    def __init__(self) -> None:
        self.poffset : POffSet = POffSet()
        self.nachi_structs : NachiStructList = NachiStructList()
        self.robotconfig : NachiConfigPosition = NachiConfigPosition()
        self.tf_exit = TransformListener()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.ros_frames_id : ROSFramesId = ROSFramesId()
        self.nachi_base : str = self.ros_frames_id.robot_base_link.replace(StringSplit.SL, str())      
        self.object_tf : str = self.ros_frames_id.object_link.replace(StringSplit.SL, str())  
        self.robot_ref_link : str = self.ros_frames_id.robot_ref_link   
        self.camera_base_link : str = self.ros_frames_id.camera_base_link      
        self.header_of_param : str = 'param_visionsetting'

    def set_object_position(self) -> GetCurrentObjectPositionResponse:   
        object_pose : GetCurrentObjectPositionResponse = GetCurrentObjectPositionResponse()
        object_pose.ack = False
        for item in self.nachi_structs.struct_list: setattr(object_pose, item, 0.0)
        return object_pose

    def euler_to_quaternion(self, rotation : Quaternion) -> EulerFromQuaternionStruct:
        euler : EulerFromQuaternionStruct = EulerFromQuaternionStruct()
        rpy = tf.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        euler.rx, euler.ry, euler.rz = rpy[0], rpy[1], rpy[2]
        return euler

    def listen_nachi_base_to_tf(self, source_frame : str) -> GetCurrentObjectPositionResponse: 
        if self.tf_exit.canTransform("/{base}".format(base=self.nachi_base) , "/{tf_name}".format(tf_name=source_frame), rospy.Time()):
            trans : TransformStamped = self.tfBuffer.lookup_transform(self.nachi_base, source_frame, rospy.Time())
            euler : EulerFromQuaternionStruct = self.euler_to_quaternion(rotation=trans.transform.rotation)
            object_pose : GetCurrentObjectPositionResponse = GetCurrentObjectPositionResponse()
            object_pose.ack = True
            for item in self.nachi_structs.pos_list: setattr(object_pose, item, getattr(trans.transform.translation, item) * 1000)
            for item in self.nachi_structs.euler_list: setattr(object_pose, item, getattr(euler, item) * 180 / math.pi)  
            return object_pose
        else:
            return self.set_object_position()

    def wait_tf_canTransform(self, target : str, source : str) -> None:
        while True:
            if self.tf_exit.canTransform(target , source, rospy.Time()):
                break

    def get_p_offset(self, model_name : str) -> GetCurrentObjectPositionResponse:
        object_pose : GetCurrentObjectPositionResponse = GetCurrentObjectPositionResponse()
        this_param : str = '/{head_param}/{name}/{param}'.format(name=model_name, head_param=self.header_of_param, param=self.poffset.name)
        if rospy.has_param(this_param):    
            object_pose.ack = True
            for item in self.nachi_structs.struct_list: setattr(object_pose, item, rospy.get_param('/{head_param}/{name}/{op}'.format(name=model_name, head_param=self.header_of_param, op = getattr(self.poffset, item))))
            return object_pose
        else:
            return self.set_object_position()

    def get_capture_table_configuration(self, model_name : str) -> GetCurrentObjectPositionResponse:
        tf_name = "{model_name}/{config}".format(model_name=model_name, config=self.robotconfig.C_T)
        if self.tf_exit.canTransform("/{base}".format(base=self.nachi_base) , "/{tf_name}".format(tf_name=tf_name), rospy.Time()):
            return self.listen_nachi_base_to_tf(source_frame=tf_name)
        else:
            return self.set_object_position()

    def get_capture_agv_configuration(self, model_name : str) -> GetCurrentObjectPositionResponse:
        tf_name = "{model_name}/{config}".format(model_name=model_name, config=self.robotconfig.C_AGV)
        if self.tf_exit.canTransform("/{base}".format(base=self.nachi_base) , "/{tf_name}".format(tf_name=tf_name), rospy.Time()):
            return self.listen_nachi_base_to_tf(source_frame=tf_name)
        else:
            return self.set_object_position()

    def get_push_agv_configuration(self, model_name : str) -> GetCurrentObjectPositionResponse:
        push_frames = self.get_all_push_agv(model_name)
        if len(push_frames) > 0:
            push_frame = self.push_agv_manipulation(push_frames, model_name)
            if push_frame != str():
                push_pose : GetCurrentObjectPositionResponse = self.listen_nachi_base_to_tf(source_frame=push_frame)
                return push_pose
            else:
                return self.set_object_position()
        else:
            return self.set_object_position()

    def get_push_table_configuration(self, model_name : str) -> GetCurrentObjectPositionResponse:
        push_frames = self.get_all_push_table(model_name)   
        if len(push_frames) > 0:
            push_frame = self.push_table_manipulation(push_frames, model_name)
            push_pose : GetCurrentObjectPositionResponse = self.listen_nachi_base_to_tf(source_frame=push_frame)
            return push_pose
        else:
            return self.set_object_position()

    def push_table_manipulation(self, push_frames, model_name : str) -> str:
        global pt_memory
        pt_filter = self.filter_memory(memory=pt_memory, name=model_name)
        if len(pt_filter) < len(push_frames):
            res = random.choice([ele for ele in push_frames if not any(x in ele for x in pt_filter)])
            pt_memory.append(res)
            pt_filter = self.filter_memory(memory=pt_memory, name=model_name)
            if len(pt_filter) == len(push_frames):
                pt_memory = [x for x in pt_memory if x not in pt_filter]
            return res
        else:
            res = random.choice([ele for ele in push_frames if not any(x in ele for x in pt_filter)])
            pt_memory.append(res)
            return res

    def push_agv_manipulation(self, push_frames, model_name : str) -> str:
        global pagv_memory
        pagv_filter = self.filter_memory(memory=pagv_memory, name=model_name)
        if len(pagv_filter) < len(push_frames):
            res = random.choice([ele for ele in push_frames if not any(x in ele for x in pagv_filter)])
            pagv_memory.append(res)
            return res
        else:
            return str()

    def can_push_agv(self, model_name : str) -> str:
        global pagv_memory  
        push_frames = self.get_all_push_agv(model_name)
        if len(push_frames) == 0:
            return self.set_object_position()
        elif len(self.filter_memory(memory=pagv_memory, name=model_name)) < len(push_frames):
            res : GetCurrentObjectPositionResponse = self.set_object_position()
            res.ack = True
            return res
        else:
            return self.set_object_position()
        
    def can_push_table(self, model_name : str) -> str:
        global pt_memory
        push_frames = self.get_all_push_table(model_name)      
        if len(push_frames) == 0:
            return self.set_object_position()
        elif len(self.filter_memory(memory=pt_memory, name=model_name)) < len(push_frames):
            res : GetCurrentObjectPositionResponse = self.set_object_position()
            res.ack = True
            return res
        else:
            return self.set_object_position()
            
    def get_all_push_table(self, model_name : str):
        counter , push_frames = 0, []
        while True:
            tf_name = "{model_name}/{config}{index_frame}".format(model_name=model_name, index_frame=counter, config=self.robotconfig.P_T)
            if self.tf_exit.canTransform("/{base}".format(base=self.nachi_base) , "/{tf_name}".format(tf_name=tf_name), rospy.Time()):
                push_frames.append(tf_name)
            else:
                break   
            counter += 1
        return push_frames

    def get_all_push_agv(self, model_name : str):
        counter , push_frames = 0, []
        while True:
            tf_name = "{model_name}/{config}{index_frame}".format(model_name=model_name, index_frame=counter, config=self.robotconfig.P_AGV)
            if self.tf_exit.canTransform("/{base}".format(base=self.nachi_base) , "/{tf_name}".format(tf_name=tf_name), rospy.Time()):
                push_frames.append(tf_name)
            else:
                break   
            counter += 1
        return push_frames

    def pull_agv_goal_position(self, model_name : str) -> GetCurrentObjectPositionResponse:
        global pagv_memory
        pagv_filter = self.filter_memory(memory=pagv_memory, name=model_name)
        if len(pagv_filter) == 0:
            return self.set_object_position()
        else:
            res = random.choice([ele for ele in pagv_filter])
            pagv_memory.remove(res)
            pull_pose : GetCurrentObjectPositionResponse = self.listen_nachi_base_to_tf(source_frame=res)
            return pull_pose

    def filter_memory(self, memory, name):
        memory_filter = []
        for item in memory:
            if (item.split('/')[0] == name):
                memory_filter.append(item)
        return memory_filter

    def set_gripper_tcp_by_model_name(self, model_name : str) -> GetCurrentObjectPositionResponse:
        current_gripper_tcp : str = '/{head_param}/{name}/gripper/tcp_config'.format(head_param=self.header_of_param, name=model_name)
        return_ack : GetCurrentObjectPositionResponse = GetCurrentObjectPositionResponse()
        if rospy.has_param(current_gripper_tcp):
            for axis in self.nachi_structs.struct_list: 
                rospy.set_param('/{head_param}/current/tcp_config/{axis}'.format(head_param=self.header_of_param, axis=axis), rospy.get_param('{current_tcp}/{axis}'.format(current_tcp=current_gripper_tcp,axis=axis))) 
            rospy.set_param('/{head_param}/current/tcp_config/name'.format(head_param=self.header_of_param), model_name)
            return_ack.ack = True
            self.wait_tf_canTransform("{rrl}_{name}".format(rrl=self.robot_ref_link, name=model_name), self.camera_base_link)
            return return_ack
        else:
            for axis in self.nachi_structs.struct_list: 
                rospy.set_param('/{head_param}/current/tcp_config/{axis}'.format(head_param=self.header_of_param, axis=axis), 0.0)   
            rospy.set_param('/{head_param}/current/tcp_config/name'.format(head_param=self.header_of_param), model_name)
            return_ack.ack = False
            self.wait_tf_canTransform("{rrl}_{name}".format(rrl=self.robot_ref_link, name=model_name), self.camera_base_link)
            return return_ack    



    

    



    



