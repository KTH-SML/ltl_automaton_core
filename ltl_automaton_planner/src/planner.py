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
from ltl_automaton_utilities import state_models_from_ts, import_ts_from_file

#Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemState, LTLPlan, LTLState, LTLStateArray
from ltl_automaton_msgs.srv import TrapCheck, TrapCheckResponse

# Import dynamic reconfigure components for dynamic parameters (see dynamic_reconfigure and dynamic_params package)
from dynamic_reconfigure.client import Client as DRClient
from dynamic_reconfigure.server import Server as DRServer
from ltl_automaton_msgs.cfg import LTLAutomatonDPConfig


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
        # init parameters, automaton, etc...
        self.init_params();

        self.build_automaton();

        self.setup_pub_sub();

        # Output plan and first command of plan
        self.publish_possible_states()
        self.publish_plan()
        self.plan_pub.publish(self.ltl_planner.next_move)
       

    def init_params(self):
        #Get parameters from parameter server
        self.agent_name = rospy.get_param('agent_name')
        self.initial_beta = rospy.get_param('initial_beta', 1000)
        self.gamma = rospy.get_param('gamma', 10)

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

        # Setup dynamic parameters (defined in dynamic_params/cfg/LTL_automaton_dynparam.cfg)
        self.re_plan_hil_param = None
        self.dynparam_srv = DRServer(LTLAutomatonDPConfig, self.dynparam_callback)

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
        state_models = state_models_from_ts(import_ts_from_file(rospy.get_param('transition_system_textfile')), self.initial_ts_dict)
     
        # Here we take the product of each element of state_models to define the full TS
        self.robot_model = TSModel(state_models)
        self.ltl_planner = LTLPlanner(self.robot_model, self.hard_task, self.soft_task, self.initial_beta, self.gamma)
        self.ltl_planner.optimal()

        # Get first value from set
        self.curr_ts_state = list(self.ltl_planner.product.graph['ts'].graph['initial'])[0]

        # initialize storage of set of possible runs in product
        self.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])

        #show_automaton(self.robot_model.product)
        #show_automaton(self.ltl_planner.product)

    #---------------------------------------------
    # Callback for checking is given TS is a trap
    #---------------------------------------------
    def trap_check_callback(self, trap_check_req):
        # Extract state from request
        ts_state = self.handle_ts_state_msg(trap_check_req.ts_state)

        # Create response message
        res = TrapCheckResponse()

        # Check if TS state is trap
        is_trap = self.ltl_planner.is_trap(ts_state)
        print is_trap

        # TS state is trap
        if is_trap == 1:
            res.is_trap = True
            res.is_connected = True
        # TS state is not a trap
        elif is_trap == -1:
            res.is_trap = False
            res.is_connected = True
        # Error: TS state is not connected to current state
        else:
            res.is_trap = False
            res.is_connected = False

        # Return service response
        return res


    def setup_pub_sub(self):
        # Prefix plan publisher
        self.prefix_plan_pub = rospy.Publisher('prefix_plan', LTLPlan, latch=True, queue_size = 1)

        # Suffix plan publisher
        self.suffix_plan_pub = rospy.Publisher('suffix_plan', LTLPlan, latch=True, queue_size = 1)

        # Possible states publisher
        self.possible_states_pub = rospy.Publisher('possible_states', LTLStateArray, latch=True, queue_size=1)

        # Initialize subscriber to provide current state of robot
        self.state_sub = rospy.Subscriber('ts_state', TransitionSystemState, self.ltl_state_callback, queue_size=1) 

        # Initialize publisher to send plan commands
        self.plan_pub = rospy.Publisher('next_move_cmd', std_msgs.msg.String, queue_size=1, latch=True)

        # Initialize check for trap service
        self.trap_srv = rospy.Service('check_for_trap', TrapCheck, self.trap_check_callback)

        # Initialize subscriber to IRL requests
        self.irl_sub = rospy.Subscriber('irl_request', std_msgs.msg.Bool, self.irl_request_callback, queue_size=1)
   

    def dynparam_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {re_plan_hil_param}""".format(**config))
        self.re_plan_hil_param = config['re_plan_hil_param']
        print('re_plan_hil_param: ' + str(self.re_plan_hil_param))
        return config

    def ltl_state_callback(self, msg=TransitionSystemState()):
        # Extract TS state from message
        state = self.handle_ts_state_msg(msg)

        is_next_state = False

        # Check if state is in TS
        #print('(in ltl_state_callback) msg state = ' + str(state))
        #print('in ltl_state_callback) robotmodel.product =' + str(self.robot_model.product.nodes()))
        if (state in self.robot_model.product.nodes()) and not(state == self.curr_ts_state):

            # Update current state
            self.curr_ts_state = state

            #-----------------------------------------------------------------------
            # Try update possible state and if error (forbidden transition), replan
            #-----------------------------------------------------------------------
            if not self.ltl_planner.update_possible_states(state):
                rospy.logerr('Can not update possible states - forbidden transition, replanning...')

                # Replan
                self.ltl_planner.replan_from_ts_state(state)
                self.publish_plan()

                # Publish new possible states
                self
                
                # Publish next move
                rospy.logwarn('Planner.py: **Re-planning** and publishing next move')
                self.plan_pub.publish(self.ltl_planner.next_move)

                return
            print "========= NEW POSSIBLE STATES =========="
            print self.ltl_planner.product.possible_states
            print "=================================="
            # Publish possible states
            self.publish_possible_states()

            #------------------------------------------------------------------
            # Try update set of possible runs and if error, display warning
            #------------------------------------------------------------------
            self.posb_runs = self.ltl_planner.update_posb_runs(self.posb_runs, state)
            if not self.posb_runs:
                print "WARNING: Empty set of possible runs"


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
                self.ltl_planner.replan_from_ts_state(state)
                self.publish_plan()

                # Publish next move
                print('Planner.py: **Re-planning** and publishing next move')
                self.plan_pub.publish(self.ltl_planner.next_move)

        elif state == self.curr_ts_state:
            rospy.logwarn("Already received state")

        elif not (state in self.robot_model.product.nodes()):
            #ERROR: unknown state (not part of TS)
            self.plan_pub.publish('None')
            rospy.logwarn('State is not in TS plan!')

    def irl_request_callback(self, msg=False):
        if msg:
            print('Planner.py **Relearning** and publishing next move')
            self.ltl_planner.irl_jit(self.posb_runs)
            # Replan
            self.ltl_planner.replan_from_ts_state(self.curr_ts_state)
            self.publish_plan()
            self.ltl_planner.find_next_move()
            self.plan_pub.publish(self.ltl_planner.next_move)

    def handle_ts_state_msg(self, ts_state_msg):
        # Extract TS state from request message
        # If only 1-dimensional state, TS graph won't use tuple, just extract the state from message array
        if len(ts_state_msg.states) > 1:
            ts_state = tuple(ts_state_msg.states)
            return ts_state
        elif len(ts_state_msg.states) == 1:
            ts_state = ts_state_msg.states[0]
            return ts_state
        else:
            raise ValueError("received empty TS state")

        #TODO Add check for message malformed (not corresponding fields)

    #----------------------------------------------
    # Publish prefix and suffix plans from planner
    #----------------------------------------------
    def publish_plan(self):
        # If plan exists
        if not (self.ltl_planner.run == None):
            # Prefix plan
            prefix_plan_msg = LTLPlan()
            prefix_plan_msg.header.stamp = rospy.Time.now()
            prefix_plan_msg.action_sequence = self.ltl_planner.run.pre_plan
            prefix_plan_msg.state_sequence = [n for n in self.ltl_planner.run.line]
            self.prefix_plan_pub.publish(prefix_plan_msg)    # publish

            # Suffix plan
            suffix_plan_msg = LTLPlan()
            suffix_plan_msg.header.stamp = rospy.Time.now()
            suffix_plan_msg.action_sequence = self.ltl_planner.run.suf_plan
            suffix_plan_msg.state_sequence = [n for n in self.ltl_planner.run.loop]
            self.suffix_plan_pub.publish(suffix_plan_msg)    # publish

    #-------------------------
    # Publish possible states
    #-------------------------
    def publish_possible_states(self):
        # Create message
        possible_states_msg = LTLStateArray()
        # For all possible state, add to the message list
        for ltl_state in self.ltl_planner.product.possible_states:
            ltl_state_msg = LTLState()
            ltl_state_msg.ts_state.states.append(ltl_state[0])
            ltl_state_msg.buchi_state = ltl_state[1]
            possible_states_msg.ltl_states.append(ltl_state_msg)

        # Publish
        self.possible_states_pub.publish(possible_states_msg)


#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('ltl_planner', anonymous=False)
    ltl_planner_node = MainPlanner()
    rospy.spin()
