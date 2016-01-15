#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from matlab_bridge.python_interface import PythonInterface
from baxter_commander import ArmCommander
from baxter_commander.persistence import dicttostate, statetodict

rospy.init_node('matlab_joint_commander')
side = 'left'
simulation = True

bridge = PythonInterface()
bridge.matlab_flag='/tmp/matlab_bridge/flagMatlabFinished.txt'
bridge.python_flag='/tmp/matlab_bridge/flagPythonFinished.txt'
bridge.matlab_file='/tmp/matlab_bridge/matlab_file.json'
bridge.python_file='/tmp/matlab_bridge/python_file.json'
joints = map(lambda joint: side+'_'+joint, ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2'])

rospy.loginfo("Starting the commander... Don't forget to roslaunch baxter_commander commander.launch if it does not end")
commander = ArmCommander(side)
rospy.loginfo("{} commander started!".format(side))
rospy.loginfo("Matlab joint commander server ready to receive commands for {}_arm {}".format(side, 'in simulation only' if simulation else 'to the real robot'))

while not rospy.is_shutdown():
    joint_values = bridge.read()
    rospy.loginfo("{}_arm command received: ".format(side) + str(joint_values))
    target = {'name': joints, 'position': list(joint_values)}
    commander.move_to_controlled(dicttostate(joint_values), display_only=simulation)
    current_angles = statetodict(commander.get_current_state())
    rospy.loginfo('Target reached! Sending back current angles {}'.format(str(current_angles)))
    bridge.send(current_angles)



