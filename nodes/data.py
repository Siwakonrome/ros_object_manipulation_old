#!/usr/bin/python3
import rospy
from ros_object_manipulation.vision_setting import main


if __name__ == "__main__":
    rospy.init_node('ros_visionsetting',anonymous=True)
    main()