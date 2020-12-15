#!/usr/bin/env python
import rospy
import yaml
from networkx.classes.digraph import DiGraph

# Import TS and action attributes from file
def import_ts_from_file(transition_system_textfile):
    try:
        # Get dictionary from yaml text file
        transition_system = yaml.load(transition_system_textfile)
        # TODO: Add a proper parse and check (dimensions, attr,...)
        return transition_system
    except:
        raise ValueError("cannot load transition system from textfile")

def state_models_from_ts(TS_dict, initial_states_dict=None):
    state_models = []

    # If initial states are given as argument
    if initial_states_dict:
        # Check that dimensions are conform
        if (len(initial_states.keys) != len(TS_dict['state_dim'])):
            raise TSError("initial states don't match TS state models: "+len(initial_states)+" initial states and "+len(TS_dict['state_dim'])+" state models")

    # For every state model define in file, using state_dim to ensure order (dict are not ordered)
    for model_dim in TS_dict['state_dim']:
        print "processing model dimension "+model_dim
        state_model_dict = TS_dict['state_models'][model_dim]
        state_model = DiGraph(initial=set(), ts_state_format=[str(model_dim)])
        #------------------
        # Create all nodes
        #------------------
        for node in state_model_dict['nodes']:
            state_model.add_node(str(node), label=set([str(node)]))
        # If no initial states in arguments, use initial state from TS dict
        if not initial_states_dict:
            state_model.graph['initial']=set([state_model_dict['initial']])
        else:
            state_model.graph['initial']=set(initial_states_dict[model_dim])
        #----------------------------------
        # Connect previously created nodes
        #----------------------------------
        # Go through all nodes
        for node in state_model_dict['nodes']:
            # Go through all connected node
            for connected_node in state_model_dict['nodes'][node]['connected_to']:
                # Add edge between node and connected node
                state_model.add_edge(str(node), str(connected_node),
                                     # Get associated action from "connected_to" tag of state node
                                     action=TS_dict['state_models'][model_dim]["nodes"][node]['connected_to'][connected_node],
                                     # Use action to retrieve weight from action dictionnary
                                     weight=TS_dict['actions'][TS_dict['state_models'][model_dim]["nodes"][node]['connected_to'][connected_node]]["weight"])
        #-------------------------
        # Add state model to list
        #-------------------------
        state_models.append(state_model)

    return state_models

def handle_ts_state_msg(ts_state_msg):
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


