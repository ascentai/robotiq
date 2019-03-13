#!/usr/bin/env python
import rospy
from robotiq_common.AdvancedController import AdvancedController
from robotiq_3f_gripper_control.msg import Robotiq3FGripper_robot_input, Robotiq3FGripper_robot_output


class ThreeFingerGripperController(AdvancedController):
    def __init__(self):
       super(ThreeFingerGripperController, self).__init__(['fingerA', 'fingerB', 'fingerC', 'scissor'], 'Robotiq3FGripperRobotInput', Robotiq3FGripper_robot_input, 'Robotiq3FGripperRobotOutput', Robotiq3FGripper_robot_output)

    def msg_from_list(self, lst):
        cmd = Robotiq3FGripper_robot_output(rACT=1, rGTO=1, rICF=1, rICS=1)
        cmd.rPRA, cmd.rSPA, cmd.rFRA, \
        cmd.rPRB, cmd.rSPB, cmd.rFRB, \
        cmd.rPRC, cmd.rSPC, cmd.rFRC, \
        cmd.rPRS, cmd.rSPS, cmd.rFRS = lst
        return cmd

    def list_from_msg(self, msg):
        return [(msg.gPOA, msg.gPRA, msg.gCUA),
                (msg.gPOB, msg.gPRB, msg.gCUB),
                (msg.gPOC, msg.gPRC, msg.gCUC),
                (msg.gPOS, msg.gPRS, msg.gCUS)
            ]

if __name__ == '__main__':
    rospy.init_node('Robotiq3FGripperAdvancedController')
    gc = ThreeFingerGripperController()
    rospy.spin()
