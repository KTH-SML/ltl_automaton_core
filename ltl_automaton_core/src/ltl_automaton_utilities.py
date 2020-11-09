import yaml
import rospkg
import os
from networkx.classes.digraph import DiGraph

def import_ts_file(file_name):
    env_file = os.path.join(rospkg.RosPack().get_path('ltl_automaton_core'), 'config', file_name)
    stream = open(env_file, 'r')
    return yaml.load(stream)

def state_models_from_ts(TS_dict):
    state_models = []
    # For every state model define in file
    for model_type in TS_dict['state_models']:
        state_model = DiGraph(initial=set())
        #------------------
        # Create all nodes
        #------------------
        for node in TS_dict['state_models'][model_type]['nodes']:
            state_model.add_node(str(node), label=set([str(node)]))
        state_model.graph['initial']=set([TS_dict['state_models'][model_type]['initial']]) 
        #----------------------------------
        # Connect previously created nodes
        #----------------------------------
        # Go through all nodes
        for node in TS_dict['state_models'][model_type]['nodes']:
            # Go through all connected node
            for connected_node in TS_dict['state_models'][model_type]['nodes'][node]['connected_to']:
                # Add edge between node and connected node
                state_model.add_edge(str(node), str(connected_node),
                                     # Get associated action from "connected_to" tag of state node
                                     action=TS_dict['state_models'][model_type]["nodes"][node]['connected_to'][connected_node],
                                     # Use action to retrieve weight from action dictionnary
                                     weight=TS_dict['actions'][TS_dict['state_models'][model_type]["nodes"][node]['connected_to'][connected_node]]["weight"])
        #-------------------------
        # Add state model to list
        #-------------------------
        state_models.append(state_model)

    return state_models


