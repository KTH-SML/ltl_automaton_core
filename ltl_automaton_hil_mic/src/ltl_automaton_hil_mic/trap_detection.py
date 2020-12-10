#!/usr/bin/env python
import rospy

class TrapDetectionPlugin(object):
	def __init__(self, args_dict):
		print "\n\n\n ========= CREATED PLUGIN =========="
		print "argument dict is"
		print args_dict
		print "============================"

	def init(self):
		print "\n\n\n INIT PLUGIN \n\n\n"

