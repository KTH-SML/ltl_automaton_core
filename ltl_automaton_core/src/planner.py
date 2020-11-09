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


def show_automaton(automaton_graph):
    pos=nx.circular_layout(automaton_graph)
    nx.draw(automaton_graph, pos)
    nx.draw_networkx_labels(automaton_graph, pos)
    edge_labels = nx.get_edge_attributes(automaton_graph, 'weight')
    nx.draw_networkx_edge_labels(automaton_graph, pos, labels = edge_labels)
    plt.show()
    return

if __name__ == '__main__':
    
    # Import TS from config file
    state_models = state_models_from_ts(import_ts_file('test_ts.yaml'))

    hard_task = '([]<> (r1 && loaded)) && ([] (r1 ->Xunloaded)) && ([]<> r2)'
    soft_task = ''
   
    # Here we take the product of each element of state_models to define the full TS
    robot_model = TSModel(state_models)
    ltl_planner = LTLPlanner(robot_model, hard_task, soft_task)
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

    # Depict the full TS and product graph
    #show_automaton(robot_model.product)
    #show_automaton(ltl_planner.product)
    
    sys.exit(0)
