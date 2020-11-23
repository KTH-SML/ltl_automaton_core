#!/usr/bin/env python
import rospy
import yaml

# Import TS and action attributes from file
def import_ts_from_file(self, transition_system_textfile):
    try:
        # Get dictionary from yaml text file
        transition_system = yaml.load(transition_system_textfile)
        # TODO: Add a proper parse and check (dimensions, attr,...)
        return transition_system
    except:
        raise ValueError("cannot load transition system from textfile")
