#!/usr/bin/env python

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 2F gripper gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' topic using the 'Robotiq2FGripper_robot_output' msg type for sending commands to a 2F gripper gripper. In this example, only the simple control mode is implemented. For using the advanced control mode, please refer to the Robotiq support website (support.robotiq.com).
"""

import rospy
import numpy as np
from std_msgs.msg import Float32
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

MIN_SPEED = 22.
MAX_SPEED = 110.
MIN_FORCE = 15.
MAX_FORCE = 60.

class Joint:
    def __init__(self, name):
        position = rospy.get_param(name + '/position', 1.0)
        speed = rospy.get_param(name + '/speed', MIN_SPEED)
        force = rospy.get_param(name + '/force', MIN_FORCE)

        self.registers = [
            int(np.clip(255 - position * 255, 0, 255)),
            int(np.clip(speed - MIN_SPEED / (MAX_SPEED - MIN_SPEED) * 255, 0, 255)),
            int(np.clip(force - MIN_FORCE / (MAX_FORCE - MIN_FORCE) * 255, 0, 255))
        ]

        self.abs_sub = rospy.Subscriber(name + '/absolute', Float32, self.abs_cb)
        self.rel_sub = rospy.Subscriber(name + '/relative', Float32, self.rel_cb)

    def abs_cb(self, msg):
        self.registers[0] = int(np.clip(255 - msg.data * 255, 0, 255))

    def rel_cb(self, msg):
        self.registers[0] = int(np.clip(self.registers[0] - msg.data * 255, 0, 255))

    def get_registers(self):
        return self.registers


if __name__ == '__main__':
    rospy.init_node('Robotiq2FGripperAdvancedController')
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
    cmd = outputMsg.Robotiq2FGripper_robot_output()

    cmd.rACT = 1
    cmd.rGTO = 1

    joint = Joint('joint')

    while not rospy.is_shutdown():
        cmd.rPR, cmd.rSP, cmd.rFR = joint.get_registers()

        pub.publish(cmd)
        rospy.sleep(0.1)
