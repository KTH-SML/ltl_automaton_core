## Define transition system
from networkx.classes.digraph import DiGraph

#----------------------------------------------
# MotionGraph: Defines motion regions and actions
# here: simple action TS with 3 regions
MotionGraph = DiGraph(initial=set())

# Define graph nodes
MotionGraph.add_node('r1', label='r1')
MotionGraph.add_node('r2', label='r2')
MotionGraph.add_node('r3', label='r3')

# Define graph edges
MotionGraph.add_edge('r1','r2', weight=10, action='goto_r2')
MotionGraph.add_edge('r2','r1', weight=10, action='goto_r1')
MotionGraph.add_edge('r1','r3', weight=10, action='goto_r3')
MotionGraph.add_edge('r3','r1', weight=10, action='goto_r1')
MotionGraph.add_edge('r3','r2', weight=10, action='goto_r2')
MotionGraph.add_edge('r2','r3', weight=10, action='goto_r3')

# Define initial state
MotionGraph.graph['initial']=set(['r1'])

#----------------------------------------------
# LoadGraph: Defines loading states and actions of pick/drop
LoadGraph = DiGraph(initial=set())

# Define graph nodes
LoadGraph.add_node('unloaded')
LoadGraph.add_node('loaded')

# Define graph edges
LoadGraph.add_edge('unloaded','loaded', weight=50, action='pick')
LoadGraph.add_edge('loaded','unloaded', weight=10, action='drop')

# Define initial state
LoadGraph.graph['initial']=set(['unloaded'])

#----------------------------------------------
# Combine graphs into list:
state_models = [MotionGraph, LoadGraph]