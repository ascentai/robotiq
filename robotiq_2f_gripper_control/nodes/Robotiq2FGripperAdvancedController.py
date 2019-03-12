#!/usr/bin/env python
import rospy
from robotiq_common.AdvancedController import AdvancedController
from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input, Robotiq2FGripper_robot_output


class TwoFingerGripperController(AdvancedController):
    def __init__(self):
        super(TwoFingerGripperController, self).__init__(['fingers'])

        self.output_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=1)

        if rospy.wait_for_message('Robotiq2FGripperRobotInput', Robotiq2FGripper_robot_input).gSTA != 3:
            self.output_pub.publish(Robotiq2FGripper_robot_output())
            rospy.sleep(0.1)

        self.output_pub.publish(self.make_cmd())
        self.input_sub = rospy.Subscriber('Robotiq2FGripperRobotInput', Robotiq2FGripper_robot_input, self.input_cb)

    def msg_from_list(self, lst):
        cmd = Robotiq2FGripper_robot_output(rACT=1, rGTO=1)
        cmd.rPR, cmd.rSP, cmd.rFR = lst
        return cmd

    def list_from_msg(self, msg):
        return [(msg.gPO, msg.gPR, msg.gCU)]


if __name__ == '__main__':
    rospy.init_node('Robotiq2FGripperAdvancedController')
    gc = TwoFingerGripperController()
    rospy.spin()
