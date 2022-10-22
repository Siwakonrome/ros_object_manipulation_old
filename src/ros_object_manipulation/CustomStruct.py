


class VisionSettingTf:
    model_name : str
    p_t : str
    p_agv : str
    c_t : str
    c_agv : str

class VisionSettingParam:
    score : str
    model_name : str
    gripper_id : str
    gripper_pos : str
    camera_params : str
    
class Offset:
    x : float 
    y : float
    z : float
    rx : float 
    ry : float
    rz : float


class EulerFromQuaternionStruct:
    rx : float 
    ry : float
    rz : float