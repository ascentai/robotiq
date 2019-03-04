#!/usr/bin/env python
import rospy
import numpy as np
import copy
from std_msgs.msg import Float32
from robotiq_common.finger_joint import FingerJoint
from robotiq_common.msg import FingerJointCmd
from robotiq_3f_gripper_control.msg import Robotiq3FGripper_robot_input, Robotiq3FGripper_robot_output


class ThreeFingerGripperController:
    def __init__(self):        
        self.joints = {name : FingerJoint(name) for name in ['fingerA','fingerB','fingerC','scissor']}
        self.output_pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripper_robot_output, queue_size=1)

        rospy.sleep(0.5)
        self.output_pub.publish(Robotiq3FGripper_robot_output())
        rospy.sleep(0.5)
        self.output_pub.publish(self.make_cmd())

        self.input_sub = rospy.Subscriber('Robotiq3FGripperRobotInput', Robotiq3FGripper_robot_input, self.input_cb)
        self.update_sub = rospy.Subscriber('finger_joint_cmds', FingerJointCmd, self.update_cb)

    def make_cmd(self):
        cmd = Robotiq3FGripper_robot_output(rACT=1, rGTO=1, rICF=1, rICS=1)
        cmd.rPRA, cmd.rSPA, cmd.rFRA = self.joints['fingerA'].get_registers()
        cmd.rPRB, cmd.rSPB, cmd.rFRB = self.joints['fingerB'].get_registers()
        cmd.rPRC, cmd.rSPC, cmd.rFRC = self.joints['fingerC'].get_registers()
        cmd.rPRS, cmd.rSPS, cmd.rFRS = self.joints['scissor'].get_registers()
        return cmd

    def update_cb(self, msg):
        for name, setpoint in zip(msg.finger_names, msg.setpoints):
            if name in self.joints:
                self.joints[name].update_setpoint(setpoint)

        self.output_pub.publish(self.make_cmd())

    def input_cb(self, msg):
        self.joints['fingerA'].publish_feedback(msg.gPOA, msg.gPRA, msg.gCUA)
        self.joints['fingerB'].publish_feedback(msg.gPOB, msg.gPRB, msg.gCUB)
        self.joints['fingerC'].publish_feedback(msg.gPOC, msg.gPRC, msg.gCUC)
        self.joints['scissor'].publish_feedback(msg.gPOS, msg.gPRS, msg.gCUS)


if __name__ == '__main__':
    rospy.init_node('Robotiq3FGripperAdvancedController')
    gc = ThreeFingerGripperController()
    rospy.spin()
