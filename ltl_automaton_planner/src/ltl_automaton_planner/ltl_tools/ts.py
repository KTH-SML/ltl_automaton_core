# -*- coding: utf-8 -*-

from ltl_automaton_planner.boolean_formulas.parser import parse as parse_guard

from math import sqrt
from networkx.classes.digraph import DiGraph

def distance(pose1, pose2):
    return (sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001)

class TSModel(DiGraph):
    #Take as input a list of state models to combine
    def __init__(self, state_models):
        self.state_models = state_models
        #self.build_full(state_models)
        #print "Full TS constructed with initial state(s):"
        #print self.graph['initial']

    def build_full(self):
        # Initialize TS model on first model from state_models
        self.product = self.state_models[0]

        # If more than one state_model, create a product graph between all models
        if (len(self.state_models) > 1):
            for sm in self.state_models[1:]:
                self.product = GraphProduct(self.product, sm)
        
        # Build class instance model from product
        DiGraph.__init__(self, 
                         incoming_graph_data=self.product,
                         initial=self.product.graph['initial'],
                         ts_state_format=self.product.graph['ts_state_format'])

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

    def fly_predecessors(self, prod_node):
        '''Return iterate of predecessors of prod_node with associated cost'''
        # IN PROGRESS
        for f_prod_node in self.product.predecessors(prod_node):
            if f_prod_node != prod_node:
                cost = self.product[f_prod_node][prod_node]['weight']
                yield f_prod_node, cost
    # # Copied from ts.py of LTL-GUI (need to debug)
    #  def fly_predecessors(self, prod_node):
    #     reg, act = self.projection(prod_node)
    #     # actions
    #     label = self.graph['region'].node[reg]['label']
    #     if act in self.graph['action'].allowed_actions(label):
    #         for f_act in self.graph['action'].action.iterkeys():
    #             f_prod_node = self.composition(reg, f_act)
    #             cost = self.graph['action'].action[act][0]
    #             self.add_edge(f_prod_node, prod_node, weight=cost, label= act)
    #             yield f_prod_node, cost
    #     # motions
    #     if act == 'None':
    #         for f_reg in self.graph['region'].predecessors(reg):
    #             if f_reg !=reg:
    #                 for f_act in self.graph['action'].action.iterkeys():
    #                         f_prod_node = self.composition(f_reg, f_act)
    #                         cost = self.graph['region'][f_reg][reg]['weight']
    #                         self.add_edge(f_prod_node, prod_node, weight=cost, label= 'goto')
    #                         yield f_prod_node, cost

    def fly_successors(self, prod_node):
        # been visited before, and hasn't changed
        if self.node[prod_node]['marker'] == 'visited':
            for prod_node_to in self.product.successors(prod_node):
                yield prod_node_to, self.product[prod_node][prod_node_to]['weight']

    # From LTL-GUI
    # def fly_successors(self, prod_node):
    #     reg, act = self.projection(prod_node)
    #     # been visited before, and hasn't changed
    #     if ((self.node[prod_node]['marker'] == 'visited') and
    #         (self.graph['region'].node[self.node[prod_node]['region']]['status'] == 'confirmed')):
    #         for prod_node_to in self.successors(prod_node):
    #             #print('in ts fly_successors visited')
    #             yield prod_node_to, self[prod_node][prod_node_to]['weight']
    #     else:
    #         #print('in ts fly_successors')
    #         self.remove_edges_from(self.out_edges(prod_node))
    #         # actions
    #         label = self.graph['region'].node[reg]['label']
    #         for act_to in self.graph['action'].allowed_actions(label):
    #             prod_node_to = self.composition(reg, act_to)
    #             cost = self.graph['action'].action[act_to][0]
    #             self.add_edge(prod_node, prod_node_to, weight=cost, label= act_to)
    #             yield prod_node_to, cost
    #         # motions
    #         for reg_to in self.graph['region'].successors(reg):
    #             #print('ts successors')
    #             #print(reg_to)
    #             #plot_automaton(self.graph['region'])
    #         #    if reg_to != reg:
    #             prod_node_to = self.composition(reg_to, 'None')
    #             cost = self.graph['region'][reg][reg_to]['weight']
    #             self.add_edge(prod_node, prod_node_to, weight=cost, label= 'goto')
    #             yield prod_node_to, cost
    #         self.graph['region'].node[self.node[prod_node]['region']]['status'] = 'confirmed'
    #         self.node[prod_node]['marker'] = 'visited'



class GraphProduct(DiGraph):
    def __init__(self, model_a, model_b):
        DiGraph.__init__(self, initial=set(), ts_state_format=model_a.graph['ts_state_format']+model_b.graph['ts_state_format'])
        self.model_a = model_a
        self.model_b = model_b
        self.do_product()

    def composition(self, state_a, state_b):
        prod_node = (state_a, state_b)
        if not self.has_node(prod_node):
            new_label = self.model_a.node[state_a]['label'].union(self.model_b.node[state_b]['label'])
            self.add_node(prod_node, label=new_label, marker='unvisited')
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