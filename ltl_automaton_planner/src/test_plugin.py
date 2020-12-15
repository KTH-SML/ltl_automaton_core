#!/usr/bin/env python
import rospy
import importlib
from geometry_msgs.msg import Pose
#from ltl_automaton_hil_mic.trap_detection import TrapDetectionPlugin

if __name__ == '__main__':
    rospy.init_node('test_node')
    test = Pose()

    plugin_dict = {'TrapDetectionPlugin': {'path': 'ltl_automaton_hil_mic.trap_detection','args': {}}}
    # If plugins are specified, try to load them
    if plugin_dict:
        plugins = {}
        for plugin in plugin_dict:
            # Import plugin module
            try:
                # Import module to a plugin dict
                plugin_module = importlib.import_module(plugin_dict[plugin]["path"])
            except ImportError:
                # Error log message
                rospy.logerr("LTL planner: Import error on loading plugin %s" % str(plugin))
                # Go to next plugin
                break

            # Get plugin class from imported module
            plugin_class = getattr(plugin_module, str(plugin))
            # Create plugin object from class using argument dictionary from parameters
            plugins.update({plugin: plugin_class(test, plugin_dict[plugin]['args'])})

    # Init plugins
    for plugin in plugins:
        plugins[plugin].init()

    test.position.x = 28.0
    test.orientation.w = 1.0

    # Init plugins
    for plugin in plugins:
        plugins[plugin].run()
