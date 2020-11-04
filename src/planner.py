#!/usr/bin/env python
import roslib
import numpy
import Queue
import rospy
from math import sqrt, cos, sin, radians
import numpy
import sys
from init import *

from ts import TSModel
from ltl_planner import LTLPlanner

import matplotlib.pyplot as plt
import networkx as nx

def show_automaton(automaton_graph):
    pos=nx.circular_layout(automaton_graph)
    nx.draw(automaton_graph, pos)
    nx.draw_networkx_labels(automaton_graph, pos)
    edge_labels = nx.get_edge_attributes(automaton_graph, 'weight')
    nx.draw_networkx_edge_labels(automaton_graph, pos, labels = edge_labels)
    plt.show()
    return

if __name__ == '__main__':
    
   
    # Here we take the product of each element of state_models to define the full TS
    # state_models defined in init.py: define components of TS 
    robot_model = TSModel(state_models)
    ltl_planner = LTLPlanner(robot_model, hard_task, soft_task)
    ltl_planner.optimal()

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
