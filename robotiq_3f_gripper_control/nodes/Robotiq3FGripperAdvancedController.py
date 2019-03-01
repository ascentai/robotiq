#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from robotiq_3f_gripper_control.msg import Robotiq3FGripper_robot_input, Robotiq3FGripper_robot_output


class Joint:
    MIN_SPEED = 22.
    MAX_SPEED = 110.
    MIN_FORCE = 15.
    MAX_FORCE = 60.

    def __init__(self, name):
        position = rospy.get_param(name + '/position', 1.0)
        speed = rospy.get_param(name + '/speed', Joint.MAX_SPEED)
        force = rospy.get_param(name + '/force', Joint.MIN_FORCE)

        self.registers = [
            int(np.clip(255 - position * 255, 0, 255)),
            int(np.clip(speed - Joint.MIN_SPEED / (Joint.MAX_SPEED - Joint.MIN_SPEED) * 255, 0, 255)),
            int(np.clip(force - Joint.MIN_FORCE / (Joint.MAX_FORCE - Joint.MIN_FORCE) * 255, 0, 255))
        ]

        self.abs_sub = rospy.Subscriber(name + '/absolute', Float32, self.abs_cb)
        self.rel_sub = rospy.Subscriber(name + '/relative', Float32, self.rel_cb)

        self.position_cmd_pub = rospy.Publisher(name + '/position_cmd', Float32, queue_size=1)
        self.position_pub = rospy.Publisher(name + '/position', Float32, queue_size=1)
        self.current_pub = rospy.Publisher(name + '/current', Float32, queue_size=1)

    def abs_cb(self, msg):
        self.registers[0] = int(np.clip(255 - msg.data * 255, 0, 255))

    def rel_cb(self, msg):
        self.registers[0] = int(np.clip(self.registers[0] - msg.data * 255, 0, 255))

    def get_registers(self):
        return self.registers

    def update(self, pos_reg, pos_cmd_reg, curr_reg):
        self.position_pub.publish(Float32(data=1.-pos_reg/255.))
        self.position_cmd_pub.publish(Float32(data=1.-pos_cmd_reg/255.))
        self.current_pub.publish(Float32(data=curr_reg*0.01))


class GripperController:
    def __init__(self):
        self.cmd = Robotiq3FGripper_robot_output(rACT=1, rGTO=1, rICF=1, rICS=1)
        self.finger_a = Joint('fingerA')
        self.finger_b = Joint('fingerB')
        self.finger_c = Joint('fingerC')
        self.scissor = Joint('scissor')
        
        self.output_pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripper_robot_output, queue_size=1)
        self.input_sub = rospy.Subscriber('Robotiq3FGripperRobotInput', Robotiq3FGripper_robot_input, self.input_cb)
        self.update_timer = rospy.Timer(rospy.Duration(1./rospy.get_param('update_hz', 10.)), self.update_cb)

    def update_cb(self, event):
        self.cmd.rPRA, self.cmd.rSPA, self.cmd.rFRA = self.finger_a.get_registers()
        self.cmd.rPRB, self.cmd.rSPB, self.cmd.rFRB = self.finger_b.get_registers()
        self.cmd.rPRC, self.cmd.rSPC, self.cmd.rFRC = self.finger_c.get_registers()
        self.cmd.rPRS, self.cmd.rSPS, self.cmd.rFRS = self.scissor.get_registers()
        self.output_pub.publish(self.cmd)

    def input_cb(self, msg):
        self.finger_a.update(msg.gPOA, msg.gPRA, msg.gCUA)
        self.finger_b.update(msg.gPOB, msg.gPRB, msg.gCUB)
        self.finger_c.update(msg.gPOC, msg.gPRC, msg.gCUC)
        self.scissor.update(msg.gPOS, msg.gPRS, msg.gCUS)


if __name__ == '__main__':
    rospy.init_node('Robotiq3FGripperAdvancedController')
    gc = GripperController()
    rospy.spin()
