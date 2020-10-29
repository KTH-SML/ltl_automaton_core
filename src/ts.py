# -*- coding: utf-8 -*-

from boolean_formulas.parser import parse as parse_guard

from math import sqrt
from networkx.classes.digraph import DiGraph

def distance(pose1, pose2):
    return (sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001)

class TSModel(DiGraph):
    #Take as input a list of state models to combine
    def __init__(self, state_models):
        self.build_full(state_models)
        print "Full TS constructed with initial state(s):"
        print self.graph['initial']

    def build_full(self, state_models):
        # Initialize TS model on first model from state_models
        self.product = state_models[0]

        # If more than one state_model, create a product graph between all models
        if (len(state_models) > 1):
            for sm in state_models[1:]:
                self.product = GraphProduct(self.product, sm)

        # Build class instance model from product
        DiGraph.__init__(self, 
                         incoming_graph_data=self.product,
                         initial=self.product.graph['initial'])

class GraphProduct(DiGraph):
    def __init__(self, model_a, model_b):
        DiGraph.__init__(self, initial=set())
        self.model_a = model_a
        self.model_b = model_b
        self.do_product()

    def composition(self, state_a, state_b):
        prod_node = (state_a, state_b)
        if not self.has_node(prod_node):
            self.add_node(prod_node, marker='unvisited')
            #If both states are initial states in their own graph, composed state is an initial state
            if (state_a in self.model_a.graph['initial']) and (state_b in self.model_b.graph['initial']):
                self.graph['initial'].add(prod_node)
        return prod_node

    #Product of two models a & b
    def do_product(self):
        #Go through all combinations of states in model a and model b
        for a_state in self.model_a.nodes():
            for b_state in self.model_b.nodes():
                #Compose node from 2 states
                prod_node = self.composition(a_state, b_state)

                for b_state_to in self.model_b.successors(b_state):
                    #=== TODO: ADD A CHECK FOR IF ACTION IS ALLOWED ON OTHER REGION STATE ===
                    prod_node_to = self.composition(a_state, b_state_to)
                    #Add edge using weight and action label from the state model
                    self.add_edge(prod_node, prod_node_to,
                                  weight=self.model_b[b_state][b_state_to]['weight'],
                                  action=self.model_b[b_state][b_state_to]['action'],
                                  marker='visited')

                for a_state_to in self.model_a.successors(a_state):
                    prod_node_to = self.composition(a_state_to, b_state)
                    #Add edge using weight and action label from the state model
                    self.add_edge(prod_node, prod_node_to,
                                  weight=self.model_a[a_state][a_state_to]['weight'],
                                  action=self.model_a[a_state][a_state_to]['action'],
                                  marker='visited')