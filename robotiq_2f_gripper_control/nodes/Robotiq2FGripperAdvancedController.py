#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from robotiq_common.finger_joint import FingerJoint
from robotiq_common.msg import FingerJointCmd
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output, Robotiq2FGripper_robot_input


class TwoFingerGripperController:
    def __init__(self):
        self.joint = FingerJoint('joint')
        self.output_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=1)

        rospy.sleep(0.5)
        self.output_pub.publish(Robotiq2FGripper_robot_output())
        rospy.sleep(0.5)
        self.output_pub.publish(self.make_cmd())

        self.input_sub = rospy.Subscriber('Robotiq2FGripperRobotInput', Robotiq2FGripper_robot_input, self.input_cb)
        self.update_sub = rospy.Subscriber('finger_joint_cmds', FingerJointCmd, self.update_cb)

    def make_cmd(self):
        cmd = Robotiq2FGripper_robot_output(rACT=1, rGTO=1)
        cmd.rPR, cmd.rSP, cmd.rFR = self.joint.get_registers()
        return cmd

    def update_cb(self, msg):
        for name, setpoint in zip(msg.finger_names, msg.setpoints):
            if name == 'joint':
                self.joint.update_setpoint(setpoint)

        self.output_pub.publish(self.make_cmd())

    def input_cb(self, msg):
        self.joint.publish_feedback(msg.gPO, msg.gPR, msg.gCU)


if __name__ == '__main__':
    rospy.init_node('Robotiq2FGripperAdvancedController')
    gc = TwoFingerGripperController()
    rospy.spin()
