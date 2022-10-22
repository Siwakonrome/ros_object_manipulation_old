import rospy



class NachiConfigPosition: 
    C_T : str = "c_t"
    P_T : str = "p_t_"
    C_AGV : str = "c_agv"
    P_AGV : str = "p_agv_"
    
class POffSet:
    x : str = 'p_offset/x'
    y : str = 'p_offset/y'
    z : str = 'p_offset/z'
    name : str = 'p_offset'
    rx : str = 'p_offset/rx'
    ry : str = 'p_offset/ry'
    rz : str = 'p_offset/rz'

class TCPOffSet:
    x : str = 'tcp_config/x'
    y : str = 'tcp_config/y'
    z : str = 'tcp_config/z'
    name : str = 'tcp_config'
    rx : str = 'tcp_config/rx'
    ry : str = 'tcp_config/ry'
    rz : str = 'tcp_config/rz'

class NachiStructList:
    pos_list = ["x", "y", "z"]
    euler_list = ["rx", "ry", "rz"]
    struct_list = ["x", "y", "z", "rx", "ry", "rz"]

class StringSplit:
    SL : str = "/"
    CM : str = ","
    OPJS : str = "{"
    CPJS : str = "}"   
    CHARP : str = "#"
    EMPTY : str = "None"

class ObjectManipulationService:
    service_list = ['object_goal_position','ct_goal_position', 'cagv_goal_position', 'poffset_goal_position', 'pt_goal_position', 'pagv_goal_position', 'can_push_agv', 'can_push_table', 'pull_agv_goal_position', 'set_current_tcp']

class ROSFramesId():
    camera_base_link_str : str = '/ros_frames_id/camera_base_link'
    robot_ref_link_str : str = '/ros_frames_id/robot_ref_link'
    object_link_str : str = '/ros_frames_id/object_link'
    robot_base_link_str : str = '/ros_frames_id/robot_base_link'
    camera_base_link_ack : bool = rospy.has_param(camera_base_link_str)
    robot_ref_link_ack : bool = rospy.has_param(robot_ref_link_str)
    object_link_ack : bool = rospy.has_param(object_link_str)
    robot_base_link_ack : bool = rospy.has_param(robot_base_link_str)
    camera_base_link : str = rospy.get_param(camera_base_link_str) if camera_base_link_ack else '/camera_base'
    robot_ref_link : str = rospy.get_param(robot_ref_link_str) if robot_ref_link_ack else '/nachi_tool'
    object_link : str = rospy.get_param(object_link_str) if object_link_ack else '/object'
    robot_base_link : str = rospy.get_param(robot_base_link_str) if robot_base_link_ack else '/base_link'