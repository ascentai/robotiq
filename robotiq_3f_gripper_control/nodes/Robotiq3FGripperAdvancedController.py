#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from robotiq_3f_gripper_control.msg import Robotiq3FGripper_robot_input, Robotiq3FGripper_robot_output, Robotiq3fGripperCmd


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


class ThreeFingerGripperController:
    def __init__(self):
        self.fingerA = GripperJoint('fingerA')
        self.fingerB = GripperJoint('fingerB')
        self.fingerC = GripperJoint('fingerC')
        self.scissor = GripperJoint('scissor')

        self.output_pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripper_robot_output, queue_size=1)
        
        if rospy.wait_for_message('Robotiq3FGripperRobotInput', Robotiq3FGripper_robot_input).gSTA != 3:
            self.output_pub.publish(Robotiq3FGripper_robot_output())
            rospy.sleep(0.1)

        self.output_pub.publish(self.make_cmd())

        self.input_sub = rospy.Subscriber('Robotiq3FGripperRobotInput', Robotiq3FGripper_robot_input, self.input_cb)
        self.update_sub = rospy.Subscriber('GripperCmd', Robotiq3fGripperCmd, self.update_cb)

    def make_cmd(self):
        cmd = Robotiq3FGripper_robot_output(rACT=1, rGTO=1, rICF=1, rICS=1)
        cmd.rPRA, cmd.rSPA, cmd.rFRA = self.fingerA.get_registers()
        cmd.rPRB, cmd.rSPB, cmd.rFRB = self.fingerB.get_registers()
        cmd.rPRC, cmd.rSPC, cmd.rFRC = self.fingerC.get_registers()
        cmd.rPRS, cmd.rSPS, cmd.rFRS = self.scissor.get_registers()
        return cmd

    def update_cb(self, msg):
        self.fingerA.update_setpoint(msg.fingerA)
        self.fingerB.update_setpoint(msg.fingerB)
        self.fingerC.update_setpoint(msg.fingerC)
        self.scissor.update_setpoint(msg.scissor)
        self.output_pub.publish(self.make_cmd())

    def input_cb(self, msg):
        self.fingerA.publish_feedback(msg.gPOA, msg.gPRA, msg.gCUA)
        self.fingerB.publish_feedback(msg.gPOB, msg.gPRB, msg.gCUB)
        self.fingerC.publish_feedback(msg.gPOC, msg.gPRC, msg.gCUC)
        self.scissor.publish_feedback(msg.gPOS, msg.gPRS, msg.gCUS)


if __name__ == '__main__':
    rospy.init_node('Robotiq3FGripperAdvancedController')
    gc = ThreeFingerGripperController()
    rospy.spin()
