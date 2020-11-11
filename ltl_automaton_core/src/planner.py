#!/usr/bin/env python
import roslib
import numpy
import Queue
import rospy
import sys
#from init import *

from ltl_tools.ts import TSModel
from ltl_tools.ltl_planner import LTLPlanner

import matplotlib.pyplot as plt
import networkx as nx
from ltl_automaton_utilities import import_ts_file, state_models_from_ts

#Import LTL automaton message definitions
import ltl_automaton_msgs.msg


def show_automaton(automaton_graph):
    pos=nx.circular_layout(automaton_graph)
    nx.draw(automaton_graph, pos)
    nx.draw_networkx_labels(automaton_graph, pos)
    edge_labels = nx.get_edge_attributes(automaton_graph, 'weight')
    nx.draw_networkx_edge_labels(automaton_graph, pos, labels = edge_labels)
    plt.show()
    return

class MainPlanner(object):
    def __init__(self):

    def init_params():
        #Get parameters from parameter server
        self.agent_name = rospy.get_param('agent_name')

        # Get LTL hard task and raise error if don't exist
        if not self.hard_task = rospy.get_param('hard_task'):
            raise InitError("Cannot initialize LTL planner, no hard_task defined")
        # Get LTL soft task
        self.soft_task = rospy.get_param('soft_task', "")

        # Parameter if initial TS is set from agent callback or from TS config file
        self.initial_ts_state_from_agent = rospy.get_param('initial_ts_state_from_agent', False)

        #If initial TS states is from agent, wait from agent state callback
        if self.initial_ts_state_from_agent:


    def build_automaton(self):
        # Import TS from config file
        state_models = state_models_from_ts(import_ts_file('test_ts.yaml'))

        hard_task = '([]<> (r1 && loaded)) && ([] (r1 ->Xunloaded)) && ([]<> r2)'
        soft_task = ''
       
        # Here we take the product of each element of state_models to define the full TS
        robot_model = TSModel(state_models)
        ltl_planner = LTLPlanner(robot_model, self.hard_task, self.soft_task)
        ltl_planner.optimal()

        show_automaton(robot_model.product)
        show_automaton(ltl_planner.product)

        # Iterate through plan to check find_next_move()
        plan_iter = 0
        plan_end = 10
        print(ltl_planner.next_move)
        while (plan_iter <= plan_end):
            print(ltl_planner.find_next_move())
            plan_iter += 1

    def setup_pub_sub(self):

    def ltl_state_callback(self, msg=TransitionSystemState()):
        
        #TODO Check if is next state in plan or not
        #if next state in plan
            # find next move
        #else if state in TS automaton
            # define init and replan
        #else
            #ERROR: unknown state (not part of TS)
            # Wait for new TS state

#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('ltl_planner',anonymous=False)
    ltl_planner_node = MainPlanner()
    rospy.spin()
