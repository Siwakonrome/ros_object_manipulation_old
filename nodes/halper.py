#!/usr/bin/python3
import rospy
from ros_object_manipulation.vision_tfhalper import VisionHalper
from ros_object_manipulation.Constant import ObjectManipulationService
from ros_object_manipulation.srv import GetCurrentObjectPositionRequest, GetCurrentObjectPositionResponse, GetCurrentObjectPosition

class NachiPositionHalper:
    def __init__(self) -> None:
        self.manipulation_service : ObjectManipulationService = ObjectManipulationService()
        self.all_halper_service = self.manipulation_service.service_list
        self.vision_halper : VisionHalper = VisionHalper()
        for item in self.all_halper_service:
            rospy.Service('object_manipulation/{service}'.format(service=item), GetCurrentObjectPosition, getattr(self, '{service}_callback'.format(service=item)))

    def pull_agv_goal_position_callback(self, req : GetCurrentObjectPositionRequest) -> GetCurrentObjectPositionResponse:
        resp : GetCurrentObjectPositionResponse = self.vision_halper.pull_agv_goal_position(req.name)
        return resp

    def can_push_agv_callback(self, req : GetCurrentObjectPositionRequest) -> GetCurrentObjectPositionResponse:
        resp : GetCurrentObjectPositionResponse = self.vision_halper.can_push_agv(req.name)
        return resp

    def can_push_table_callback(self, req : GetCurrentObjectPositionRequest) -> GetCurrentObjectPositionResponse:
        resp : GetCurrentObjectPositionResponse = self.vision_halper.can_push_table(req.name)
        return resp

    def object_goal_position_callback(self, req : GetCurrentObjectPositionRequest) -> GetCurrentObjectPositionResponse:   
        resp : GetCurrentObjectPositionResponse = self.vision_halper.listen_nachi_base_to_tf_at_time()
        return resp

    def ct_goal_position_callback(self, req : GetCurrentObjectPositionRequest) -> GetCurrentObjectPositionResponse:
        resp : GetCurrentObjectPositionResponse = self.vision_halper.get_capture_table_configuration(req.name)
        return resp

    def cagv_goal_position_callback(self, req : GetCurrentObjectPositionRequest) -> GetCurrentObjectPositionResponse:
        resp : GetCurrentObjectPositionResponse = self.vision_halper.get_capture_agv_configuration(req.name)
        return resp

    def poffset_goal_position_callback(self, req : GetCurrentObjectPositionRequest) -> GetCurrentObjectPositionResponse:
        resp : GetCurrentObjectPositionResponse = self.vision_halper.get_p_offset(req.name)
        return resp

    def pt_goal_position_callback(self, req : GetCurrentObjectPositionRequest) -> GetCurrentObjectPositionResponse:
        resp : GetCurrentObjectPositionResponse = self.vision_halper.get_push_table_configuration(req.name)
        return resp

    def pagv_goal_position_callback(self, req : GetCurrentObjectPositionRequest) -> GetCurrentObjectPositionResponse:
        resp : GetCurrentObjectPositionResponse = self.vision_halper.get_push_agv_configuration(req.name)
        return resp 

    def set_current_tcp_callback(self, req : GetCurrentObjectPositionRequest) -> GetCurrentObjectPositionResponse:
        resp : GetCurrentObjectPositionResponse = self.vision_halper.set_gripper_tcp_by_model_name(req.name)
        return resp 

def main():
    rospy.init_node('object_manipulation_halper')
    NachiPositionHalper()
    rospy.spin()


if __name__ == "__main__":
    main()