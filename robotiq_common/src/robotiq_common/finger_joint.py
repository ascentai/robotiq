#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32


class FingerJoint:
    MIN_SPEED = 22.
    MAX_SPEED = 110.
    MIN_FORCE = 15.
    MAX_FORCE = 60.

    def __init__(self, name):
        position = rospy.get_param(name + '/position', 1.0)
        speed = rospy.get_param(name + '/speed', FingerJoint.MAX_SPEED)
        force = rospy.get_param(name + '/force', FingerJoint.MIN_FORCE)
        self.relative = rospy.get_param(name + '/relative_setpoints', True)

        self.registers = [
            int(np.clip(255 - position * 255, 0, 255)),
            int(np.clip(speed - FingerJoint.MIN_SPEED / (FingerJoint.MAX_SPEED - FingerJoint.MIN_SPEED) * 255, 0, 255)),
            int(np.clip(force - FingerJoint.MIN_FORCE / (FingerJoint.MAX_FORCE - FingerJoint.MIN_FORCE) * 255, 0, 255))
        ]

        self.position_cmd_pub = rospy.Publisher(name + '/position_cmd', Float32, queue_size=1)
        self.position_pub = rospy.Publisher(name + '/position', Float32, queue_size=1)
        self.current_pub = rospy.Publisher(name + '/current', Float32, queue_size=1)

    def update_setpoint(self, sp_reg):
        offset = self.registers[0] if self.relative else 255
        self.registers[0] = int(np.clip(offset - sp_reg * 255, 0, 255))

    def publish_feedback(self, pos_reg, pos_cmd_reg, curr_reg):
        self.position_pub.publish(Float32(data=1.-pos_reg/255.))
        self.position_cmd_pub.publish(Float32(data=1.-pos_cmd_reg/255.))
        self.current_pub.publish(Float32(data=curr_reg*0.01))

    def get_registers(self):
        return self.registers