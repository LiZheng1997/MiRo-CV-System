import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import logging
import os
import sys
import rospy
import miro2 as miro
from utils import data_resolver
import pars
from client_control_module import ControlNode



if __name__ == "__main__":
    # init ROS
    rospy.init_node(os.getenv("MIRO_ROBOT_NAME") + "_client_demo") # log_level=self.pars.ros.log_level
    print (os.getenv("MIRO_ROBOT_NAME") + "_client_demo")
    pars = pars.CorePars()
    topic_base_name = "/" + pars.ros.robot_name + "/"
    
    print("The robot name is ",topic_base_name)
    main = ControlNode("miro")
    logging.info("Initiate the robot named miro")
    main.init_kinematic()
    logging.info("Initiate the kinematic status of MiRo")

    data_resolver = data_resolver.DataResolver()

    ###########################################################################
    # Module Test codes, after all modules have been tested, we can integrate #
    # them into the main client module.										  #
    ###########################################################################

    # main.init_movement_detection()
    # main.init_pedestrain_detection()
    # main.init_NN_detection_caffe_l()
    # main.init_NN_detection_tensorflow_mask_rcnn_l()
    # main.init_NN_detection_tensorflow_mobile_ssd_v2_l()
    # main.init_NN_detection_tensorflow_mobile_ssd_v1_l()
    # main.init_match_target()
    # main.test()
    # main.init_path_planning()
    # main.multiprocess_ball()
    # main.init_miro_detection_l()
    # main.init_miro_detection_r()
    # main.init_ball_detection_l()
    # main.init_ball_detection_r()