## Define transition system
from networkx.classes.digraph import DiGraph

#----------------------------------------------
# MotionGraph: Defines motion regions and actions
# here: simple action TS with 3 regions
MotionGraph = DiGraph(initial=set())

# Define graph nodes
MotionGraph.add_node('r1', label=set(['r1']))
MotionGraph.add_node('r2', label=set(['r2 && !loaded']))
MotionGraph.add_node('r3', label=set(['r3']))

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
LoadGraph.add_node('unloaded', label=set(['unloaded']))
LoadGraph.add_node('loaded', label=set(['loaded']))

# Define graph edges
LoadGraph.add_edge('unloaded','loaded', weight=50, action='pick')
LoadGraph.add_edge('loaded','unloaded', weight=10, action='drop')

# Define initial state
LoadGraph.graph['initial']=set(['unloaded'])

#--------------------------------------------
# # BatteryGraph: Defines battery state
# BatteryGraph = DiGraph(initial=set())

# # Define graph nodes
# BatteryGraph.add_node('battery_low', label='battery_low')
# BatteryGraph.add_node('battery_high', label='battery_high')

# # Define graph edges
# BatteryGraph.add_edge('battery_low','battery_high', weight=10, action='charge')

# # Define initial state
# LoadGraph.graph['initial']=set(['battery_high'])


#----------------------------------------------
# Combine graphs into list:
state_models = [MotionGraph, LoadGraph]

#----------------------------------------------
# LTL formula and buchi generation
#hard_task = '([]<> r1) && ([]<> r2)'
#hard_task = '([]<> loaded ) && ([]<> unloaded )'
#hard_task = '([]<> r1) && ([]<> r2) && ([] (r2 -> Xr3))'
hard_task = '([]<> (r1 && loaded)) && ([] (r1 ->Xunloaded)) && ([]<> r2)'
soft_task = ''

