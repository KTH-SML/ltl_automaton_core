#!/usr/bin/env python
import roslib
import numpy
import Queue
import rospy
from math import sqrt, cos, sin, radians
import numpy
import sys

from ts import MotionStateModel, LoadStateModel, TSModel

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
    motion_model = MotionStateModel()
    load_state_model = LoadStateModel()
    model = TSModel([motion_model, load_state_model])
    show_automaton(model.product)
    sys.exit(0)
