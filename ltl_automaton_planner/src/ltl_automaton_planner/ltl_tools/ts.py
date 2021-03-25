# -*- coding: utf-8 -*-
import rospy
from ltl_automaton_planner.boolean_formulas.parser import parse as parse_guard

from math import sqrt
from itertools import product
from networkx.classes.digraph import DiGraph

import networkx as nx

def distance(pose1, pose2):
    return (sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001)


def node_product(*args):
    node_pools = [list(pool) for pool in args]
    product_pool = [tuple()]
    for node_pool in node_pools:
        product_pool = [x+y for x in product_pool for y in node_pool]

    return product_pool

class TSModel(DiGraph):
    #Take as input a list of state models to combine
    def __init__(self, state_models):
        self.state_models = state_models
        #self.build_full(state_models)
        #print("Full TS constructed with initial state(s):")
        #print(self.graph['initial'])

    def build_full(self):
        # Initialize TS model on first model from state_models
        self.product = self.state_models[0]

        for sm in self.state_models:
            print("-------- NODES ARE --------")
            print(sm.nodes(data=True))
            print("-------- INITIAL NODES ARE --------")
            print(list(sm.graph['initial']))
            

        print("-------- PRODUCT NODES ARE --------")
        print(node_product(*self.state_models))
        print("-------- ================= --------")

        # If more than one state_model, create a product graph between all models
        # if (len(self.state_models) > 1):
        #     for sm in self.state_models[1:]:
        #         self.product = GraphProduct(self.product, sm)

        self.product = GraphProduct(self.state_models)
        
        # Build class instance model from product
        DiGraph.__init__(self, 
                         incoming_graph_data=self.product,
                         initial=self.product.graph['initial'],
                         ts_state_format=self.product.graph['ts_state_format'])
        rospy.loginfo("LTL Planner: full model constructed with %d states and %s transitions" %(len(self.nodes()), len(self.edges())))
        rospy.loginfo("LTL Planner: initial state in TS is %s" %str(self.graph['initial']))

        print("-------- NODES ARE --------")
        print(self.nodes(data=True))
        print("-------- INITIAL NODES ARE --------")
        print(self.graph['initial'])

    #----------------------------------
    # Delete and set new initial state
    #----------------------------------
    def set_initial(self, ts_state):
        # If state exist in graph, change initial and return true
        if ts_state in self.product.nodes():
            self.graph['initial'] = set([ts_state])
            return True
        # If state doesn't exist in graph, return false
        else:
            return False

class GraphProduct(DiGraph):
    # def __init__(self, model_a, model_b):
    #     #DiGraph.__init__(self, initial=set(), ts_state_format=[model.graph['ts_state_format'] for model in graph_list])
    #     DiGraph.__init__(self, initial=set(), ts_state_format=model_a.graph['ts_state_format']+model_b.graph['ts_state_format'])
    #     print("==== TS STATE FORMAT ====")
    #     print(self.graph['ts_state_format'])
    #     self.model_a = model_a
    #     self.model_b = model_b
    #     self.do_product()
    def __init__(self, graph_list):
        DiGraph.__init__(self, initial=set(), ts_state_format=[model.graph['ts_state_format'] for model in graph_list])
        self.compose_nodes(graph_list)
        self.compose_edges(graph_list)
        self.compose_initial(graph_list)


    def compose_initial(self, graph_list):
        # init_node = ()
        # for graph in graph_list:
        #     print(":::::: INiti node :::::")
        #     print(init_node)
        #     print(":::::: graph init ::::::")
        #     print(graph.graph['initial'])
        #     for init_nodes
        #     init_node = init_node + graph.graph['initial']

        initial_states = [list(graph.graph['initial']) for graph in graph_list]
        print(":::::: graph init ::::::")
        print(initial_states)
        init_nodes = self.node_product(*initial_states)
        print(":::::: INiti node :::::")
        print(init_nodes)

        self.graph['initial'].update(set(init_nodes))

    def compose_nodes(self, graph_list):
        node_product = self.node_product(*graph_list)
        for node in node_product:
            self.add_node(node, label=str(node), marker='unvisited')

    def compose_edges(self, graph_list):
        # For each individual state model
        for i in range(len(graph_list)):
            # For each state in this model
            for state in graph_list[i]:
                # Look for node in the product which include this state
                nodes = [elem for elem in self.nodes if elem[i] == state[0]]
                print("Composing nodes for state model number %i and state %s" % (i, state))
                for node in nodes:
                    print("EDGE COMPOSISING NODE IS")
                    print(node)
                    successor_state_node = list(node)
                    for successor_state in graph_list[i].successors(state):
                        # Create successor node by replacing one state by its successor
                        successor_state_node[i] = successor_state[0]
                        successor_node = tuple(successor_state_node)
                        print("SUCCESSOR NODE IS")
                        print(successor_node)
                        #Add edge using weight and action label from the state model
                        if self.is_action_allowed(graph_list[i][state][successor_state]['guard'], self.nodes[node]['label']):
                            print("Action %s allowed with guard %s against label %s" % (graph_list[i][state][successor_state]['action'], graph_list[i][state][successor_state]['guard'], self.nodes[node]['label']))
                            self.add_edge(node, successor_node,
                                          action=graph_list[i][state][successor_state]['action'],
                                          guard=graph_list[i][state][successor_state]['guard'],
                                          weight=graph_list[i][state][successor_state]['weight'],
                                          marker='visited')
                        else:
                            print("Action %s IS NOT allowed with guard %s against label %s" % (graph_list[i][state][successor_state]['action'], graph_list[i][state][successor_state]['guard'], self.nodes[node]['label']))



    def composition(self, state_a, state_b):
        prod_node = state_a + state_b
        if not self.has_node(prod_node):
            new_label = self.model_a.nodes[state_a]['label'].union(self.model_b.nodes[state_b]['label'])
            self.add_node(prod_node, label=new_label, marker='unvisited')
            #If both states are initial states in their own graph, composed state is an initial state
            if (state_a in self.model_a.graph['initial']) and (state_b in self.model_b.graph['initial']):
                self.graph['initial'].add(prod_node)
        return prod_node

    def is_action_allowed(self, action_guard, ts_label):
        # Check action guard against the node label
        guard_expr = parse_guard(action_guard)
        if guard_expr.check(ts_label):
            return True
        else:
            return False


    #Product of two models a & b
    def do_product(self):
        #Go through all combinations of states in model a and model b
        for a_state in self.model_a.nodes():
            for b_state in self.model_b.nodes():
                #Compose node from 2 states
                prod_node = self.composition(a_state, b_state)

                for b_state_to in self.model_b.successors(b_state):
                    prod_node_to = self.composition(a_state, b_state_to)
                    #Add edge using weight and action label from the state model
                    if self.is_action_allowed(self.model_b[b_state][b_state_to]['guard'], self.model_a.nodes[a_state]['label']):
                        self.add_edge(prod_node, prod_node_to,
                                      action=self.model_b[b_state][b_state_to]['action'],
                                      guard=self.model_b[b_state][b_state_to]['guard'],
                                      weight=self.model_b[b_state][b_state_to]['weight'],
                                      marker='visited')

                for a_state_to in self.model_a.successors(a_state):
                    prod_node_to = self.composition(a_state_to, b_state)
                    #Add edge using weight and action label from the state model
                    if self.is_action_allowed(self.model_a[a_state][a_state_to]['guard'], self.model_b.nodes[b_state]['label']):
                        self.add_edge(prod_node, prod_node_to,
                                      action=self.model_a[a_state][a_state_to]['action'],
                                      guard=self.model_a[a_state][a_state_to]['guard'],
                                      weight=self.model_a[a_state][a_state_to]['weight'],
                                      marker='visited')
    @staticmethod
    def node_product(*args):
        node_pools = [list(pool) for pool in args]
        product_pool = [tuple()]
        for node_pool in node_pools:
            product_pool = [x+y for x in product_pool for y in node_pool]

        return product_pool

    @staticmethod
    def init_product(*args):
        node_pools = [list(pool) for pool in args]
        product_pool = [tuple()]
        for node_pool in node_pools:
            product_pool = [x+y for x in product_pool for y in node_pool]

        return product_pool