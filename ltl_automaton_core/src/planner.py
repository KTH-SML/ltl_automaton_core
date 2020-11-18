#!/usr/bin/env python
import roslib
import numpy
import Queue
import rospy
import sys
#from init import *
import yaml

import std_msgs

from ltl_tools.ts import TSModel
from ltl_tools.ltl_planner import LTLPlanner

import matplotlib.pyplot as plt
import networkx as nx
from ltl_automaton_utilities import import_ts_file, state_models_from_ts

#Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemState


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

        self.init_params();

        self.build_automaton();

        self.setup_pub_sub();

        # Output first command of plan
        self.plan_pub.publish(self.ltl_planner.next_move)
       

    def init_params(self):
        #Get parameters from parameter server
        self.agent_name = rospy.get_param('agent_name')

        # Get TS from param
        transition_system_textfile = rospy.get_param('transition_system_textfile')
        self.transition_system = yaml.load(transition_system_textfile)
        print self.transition_system

        # Get LTL hard task and raise error if don't exist
        if (rospy.has_param('hard_task')):
            self.hard_task = rospy.get_param('hard_task')
        else:
            raise InitError("Cannot initialize LTL planner, no hard_task defined")

        # Get LTL soft task
        self.soft_task = rospy.get_param('soft_task', "")

        # Parameter if initial TS is set from agent callback or from TS config file
        self.initial_ts_state_from_agent = rospy.get_param('initial_ts_state_from_agent', False)

        #If initial TS states is from agent, wait from agent state callback
        if self.initial_ts_state_from_agent:
            self.initial_ts_dict = init_ts_state_from_agent(rospy.wait_for_message("ts_state", TransitionSystemState))
        else:
            self.initial_ts_dict = None

    # 
    def init_ts_state_from_agent(self, msg=TransitionSystemState):
        initial_ts_dict_ = None

        # If message is conform (same number of state as number of state dimensions)
        if (len(msg.states) == len(msg.state_dimension_names)):
            # Create dictionnary with paired dimension_name/state_value
            initial_ts_dict_ = dict()
            for i in range(msg.states):
                initial_ts_dict_.append({msg.state_dimension_names[i] : msg.states[i]}) 

        # Else message is malformed, raise error
        else:
            raise TSError("initial states don't match TS state models: "+len(msg.states)+" initial states and "+len(msg.state_dimension_names)+" state models")
        
        # Return initial state dictionnary
        return initial_ts_dict_


    def build_automaton(self):
        # Import TS from config file
        state_models = state_models_from_ts(import_ts_file('test_ts.yaml'), self.initial_ts_dict)
     
        # Here we take the product of each element of state_models to define the full TS
        self.robot_model = TSModel(state_models)
        self.ltl_planner = LTLPlanner(self.robot_model, self.hard_task, self.soft_task)
        self.ltl_planner.optimal()

        #show_automaton(self.robot_model.product)
        #show_automaton(self.ltl_planner.product)

        

        # # Iterate through plan to check find_next_move()
        # plan_iter = 0
        # plan_end = 10
        # print(ltl_planner.next_move)
        # while (plan_iter <= plan_end):
        #     print(ltl_planner.find_next_move())
        #     plan_iter += 1

    def setup_pub_sub(self):

        # Initialize subscriber to provide current state of robot
        self.state_sub = rospy.Subscriber('ts_state', TransitionSystemState , self.ltl_state_callback, queue_size=1) 

        # Initialize publisher to send plan commands
        self.plan_pub = rospy.Publisher('next_move_cmd', std_msgs.msg.String, queue_size=1, latch=True)


    def ltl_state_callback(self, msg=TransitionSystemState()):
        
        # Get system state, convert to tuple and set boolean condition to False
        state = tuple(msg.states)
        #state = ('unloaded','r2') # FOR DEBUGGING REMOVE
        is_next_state = False

        # Check if state is in TS
        #print('(in ltl_state_callback) msg state = ' + str(state))
        #print('in ltl_state_callback) robotmodel.product =' + str(self.robot_model.product.nodes()))
        if state in self.robot_model.product.nodes():

            #print('in ltl_state_callback):  self.ltl_planner.segent = ' + str(self.ltl_planner.segment))

            # Check if plan is in prefix or suffix phase
            if self.ltl_planner.segment == 'line':

                #print('in ltl_state_callback) ltl_planner.run.line[ltl_planner.index] = ' + str(self.ltl_planner.run.line[self.ltl_planner.index]))

                # Check if state is the next state of the plan
                if state == self.ltl_planner.run.line[self.ltl_planner.index+1]:
                    is_next_state = True

            if self.ltl_planner.segment == 'loop':
                # Check if state is the next state of the plan
                if state == self.ltl_planner.run.loop[self.ltl_planner.index+1]:
                    is_next_state = True

            # If state is next state in plan, find next_move and output
            if is_next_state:
                self.ltl_planner.find_next_move()

                # Publish next move
                print('Planner.py: Publishing next move')
                self.plan_pub.publish(self.ltl_planner.next_move)
               
            elif not(is_next_state):
                #Set state as initial

                # Replan
                self.ltl_planner.replan()

                # Publish next move
                print('Planner.py: **Re-planning** and publishing next move')
                self.plan_pub.publish(self.ltl_planner.next_move)


        else:
            #ERROR: unknown state (not part of TS)
            self.plan_pub.publish('None')
            rospy.logwarn('State is not in TS plan!')

#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('ltl_planner',anonymous=False)
    ltl_planner_node = MainPlanner()
    rospy.spin()
