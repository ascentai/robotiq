#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output, Robotiq2FGripper_robot_input


class GripperJoint:
    MIN_SPEED = 22.
    MAX_SPEED = 110.
    MIN_FORCE = 15.
    MAX_FORCE = 60.

    def __init__(self, name):
        position = rospy.get_param(name + '/position', 1.0)
        speed = rospy.get_param(name + '/speed', GripperJoint.MAX_SPEED)
        force = rospy.get_param(name + '/force', GripperJoint.MIN_FORCE)

        self.registers = [
            int(np.clip(255 - position * 255, 0, 255)),
            int(np.clip(speed - GripperJoint.MIN_SPEED / (GripperJoint.MAX_SPEED - GripperJoint.MIN_SPEED) * 255, 0, 255)),
            int(np.clip(force - GripperJoint.MIN_FORCE / (GripperJoint.MAX_FORCE - GripperJoint.MIN_FORCE) * 255, 0, 255))
        ]

        self.position_cmd_pub = rospy.Publisher(name + '/position_cmd', Float32, queue_size=1)
        self.position_pub = rospy.Publisher(name + '/position', Float32, queue_size=1)
        self.current_pub = rospy.Publisher(name + '/current', Float32, queue_size=1)

    def update_setpoint(self, sp_reg):
        self.registers[0] = int(np.clip(255 - sp_reg * 255, 0, 255))

    def publish_feedback(self, pos_reg, pos_cmd_reg, curr_reg):
        self.position_pub.publish(Float32(data=1.-pos_reg/255.))
        self.position_cmd_pub.publish(Float32(data=1.-pos_cmd_reg/255.))
        self.current_pub.publish(Float32(data=curr_reg*0.01))

    def get_registers(self):
        return self.registers


class TwoFingerGripperController:
    def __init__(self):
        self.joint = GripperJoint('joint')

        self.output_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=1)

        if rospy.wait_for_message('Robotiq2FGripperRobotInput', Robotiq2FGripper_robot_input).gSTA != 3:
            self.output_pub.publish(Robotiq2FGripper_robot_output())
            rospy.sleep(0.1)

        self.output_pub.publish(self.make_cmd())

        self.input_sub = rospy.Subscriber('Robotiq2FGripperRobotInput', Robotiq2FGripper_robot_input, self.input_cb)
        self.update_sub = rospy.Subscriber('GripperCmd', Float32, self.update_cb)

    def make_cmd(self):
        cmd = Robotiq2FGripper_robot_output(rACT=1, rGTO=1)
        cmd.rPR, cmd.rSP, cmd.rFR = self.joint.get_registers()
        return cmd

    def update_cb(self, msg):
        self.joint.update_setpoint(msg.data)
        self.output_pub.publish(self.make_cmd())

    def input_cb(self, msg):
        self.joint.publish_feedback(msg.gPO, msg.gPR, msg.gCU)


if __name__ == '__main__':
    rospy.init_node('Robotiq2FGripperAdvancedController')
    gc = TwoFingerGripperController()
    rospy.spin()
