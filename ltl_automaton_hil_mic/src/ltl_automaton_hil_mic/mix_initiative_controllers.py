#!/usr/bin/env python
import rospy

#==============================================
#         Mix Initative Controller object
#            for Velocity Commands
#                     ---
# Takes as input velocity commands (Twist msg)
# and a distance to trap metric to mix planner
#             and human inputs
#==============================================
class VelCmdMixer(object):
    def __init__(self, planner_input_topic, human_input_topic, mix_output_topic):

    def init_params(self):

    def set_pub_sub(self):


#=============================================
#       Mix Initative Controller object
#            for Action Commands
#                     ---
#  Take as input an action name (String msg)
#  to test the potential resulting state as
#   trap or not and output either planner or
#           human action command
#=============================================
class ActionCmdMixer(object):


#=============================================
#       Mix Initative Controller object
#            for action commands
#                     ---
#  Take as input an action name (String msg)
#  to test the potential resulting state as
#   trap or not and output either planner or
#           human action command
#=============================================
class BoolCmdMixer(object):
