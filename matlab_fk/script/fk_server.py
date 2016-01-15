#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from matlab_bridge.python_interface import PythonInterface
import baxter_pykdl

rospy.init_node('matlab_fk_server')
side = 'left'

kinematics = baxter_pykdl.baxter_kinematics(side)
bridge = PythonInterface()
bridge.matlab_flag='/tmp/matlab_bridge/FKmatlabflag.txt'
bridge.python_flag='/tmp/matlab_bridge/FKpythonflag.txt'
bridge.matlab_file='/tmp/matlab_bridge/FKjoints.json'
bridge.python_file='/tmp/matlab_bridge/FKxyz.json'
joints = map(lambda joint: side+'_'+joint, ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2'])

rospy.loginfo("FK server ready to handle FK for {}_arm".format(side))

while not rospy.is_shutdown():
    joint_values = bridge.read()
    rospy.loginfo(" {}_arm FK request: ".format(side) + str(joint_values))
    fk_request = dict(zip(joints, joint_values))
    fk_result = kinematics.forward_position_kinematics(fk_request)
    position = list(fk_result[:3])
    orientation = list(fk_result[3:])
    bridge.send(position)



