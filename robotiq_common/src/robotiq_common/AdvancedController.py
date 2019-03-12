#!/usr/bin/env python
import rospy
import numpy as np
from abc import abstractmethod
from sensor_msgs import JointState


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

    def parse_feedback(self, pos_reg, pos_cmd_reg, curr_reg):
        self.position = 1. - pos_reg / 255.
        self.position_cmd_echo = 1. - pos_cmd_reg / 255.
        self.current = curr_reg * 0.01

    def update_setpoint(self, msg):
        self.registers[0] = int(np.clip(255 - msg.data * 255, 0, 255))

    def get_registers(self):
        return self.registers


class AdvancedController:
    def __init__(self, joint_names):
        self.joint_names = joint_names
        self.joints = {name: GripperJoint(name) for name in self.joint_names}
        self.feedback_pub = rospy.Publisher('Feedback', JointState, queue_size=1)
        self.update_sub = rospy.Subscriber('GripperCmd', JointState, self.update_cb)  
        self.output_pub = None

    @abstractmethod
    def msg_from_list(self, lst):
        pass

    @abstractmethod
    def list_from_msg(self, msg):
        pass

    def make_cmd(self):
        return self.msg_from_list([reg for reg in self.joints[name].get_registers() for name in self.joint_names])

    def update_cb(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.joints[name].update_setpoint(position)

        self.output_pub.publish(self.make_cmd())

    def input_cb(self, msg):
        joint_state_msg = JointState()

        for name, reg in zip(self.joint_names, self.list_from_msg(msg)):
            self.joints[name].parse_feedback(*reg)
            joint_state_msg.name.append(name)
            joint_state_msg.position.append(self.joints[name].position)
            joint_state_msg.effort.append(self.joints[name].current)

        self.feedback_pub.publish(joint_state_msg)

