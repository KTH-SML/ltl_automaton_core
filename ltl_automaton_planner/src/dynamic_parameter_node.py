#!/usr/bin/env python
import rospy
import sys
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from dynamic_params.cfg import DPConfig

# this is used to test/exemplify the dynamic parameter by updating a change of the re_plan_hil_param 
# used in the ltl_planner node
class DynamicParameter(object):
    def __init__(self):

        # Define node containing dynamic parameter to change
        self.dyn_node = 'ltl_planner'
        self.client = Client(self.dyn_node)

        print('---------------')
        print('Changing parameter re_plan_hil_param!')
        params = { 're_plan_hil_param' : False}
        config = self.client.update_configuration(params)

        self.main_loop()

    def main_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # To Do
            
            rate.sleep()    

#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('dynamic_parameter',anonymous=False)
    try:
        dynparam = DynamicParameter()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("Dynamic Parameter node: %s" %(e))
        sys.exit(0)