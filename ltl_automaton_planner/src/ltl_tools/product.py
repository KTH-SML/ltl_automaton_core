# -*- coding: utf-8 -*-

from buchi import check_label_for_buchi_edge
#from discrete_plan import has_path_to_accept

from networkx.classes.digraph import DiGraph


class ProdAut(DiGraph):
    def __init__(self, ts, buchi, alpha=1000):
        DiGraph.__init__(self, ts=ts, buchi=buchi, alpha=alpha, initial=set(), accept=set(), type='ProdAut')

    # Build product automaton of TS and büchi by exploring every node combination and adding edges when required
    def build_full(self):
        # Iterate over all TS nodes and buchi nodes
        for f_ts_node in self.graph['ts'].nodes():
            for f_buchi_node in self.graph['buchi'].nodes():
                # Compose node from current TS and buchi node
                f_prod_node = self.composition(f_ts_node, f_buchi_node)
                # Iterate over all nodes connected to current TS node and büchi node
                for t_ts_node in self.graph['ts'].successors(f_ts_node):
                    for t_buchi_node in self.graph['buchi'].successors(f_buchi_node):
                            # Compose node from connected buchi node and connected TS node, and check if this node
                            # should be connected to previously composed TS/büchi node
                            t_prod_node = self.composition(t_ts_node, t_buchi_node)
                            # Get label from TS node, and weight and action from TS edge
                            label = self.graph['ts'].node[f_ts_node]['label']
                            cost = self.graph['ts'][f_ts_node][t_ts_node]['weight']
                            action = self.graph['ts'][f_ts_node][t_ts_node]['action']
                            # Check if label is compatible with büchi (black magic for now, need to understand this better)
                            truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
                            total_weight = cost + self.graph['alpha']*dist
                            if truth:
                                self.add_edge(f_prod_node, t_prod_node, weight=total_weight, action=action)

        # Build initial reachable set from initial state
        self.reachable_states = set(self.graph['initial'])
        #self.reachable_states = self.update_reachable(self.graph['ts'].graph['initial'])

    def composition(self, ts_node, buchi_node):
        # Compose node from TS and Büchi
        prod_node = (ts_node, buchi_node)
        # If node not already in product graph, add node to graph
        if not self.has_node(prod_node):
            self.add_node(prod_node, ts=ts_node, buchi=buchi_node, marker='unvisited')
            # If TS and Büchi nodes are both initial nodes in their own graph, composed node is initial
            if ((ts_node in self.graph['ts'].graph['initial']) and
                (buchi_node in self.graph['buchi'].graph['initial'])):
                self.graph['initial'].add(prod_node)
            # If Büchi node is an accept state, composed node is an accept state
            if (buchi_node in self.graph['buchi'].graph['accept']):
                self.graph['accept'].add(prod_node)
        return prod_node

    def projection(self, prod_node):
        ts_node = self.node[prod_node]['ts']
        buchi_node = self.node[prod_node]['buchi']
        return ts_node, buchi_node

    #------------------------------
    # Build initial product states
    #------------------------------
    # Initial states of TS and Büchi needs to be
    # defined before calling this function
    def build_initial(self):
        # Reset initial set
        self.graph['initial'] = set()
        # Go through all initial states in TS
        for ts_init in self.graph['ts'].graph['initial']:
            # Go through all initial states in Büchi
            for buchi_init in self.graph['buchi'].graph['initial']:
                # If both TS and Büchi state are initial state, build composed node and add it to the product initial set
                init_prod_node = (ts_init, buchi_init)
                self.graph['initial'].add(init_prod_node)

        # Build initial reachable set from initial state
        self.reachable_states = set(self.graph['initial'])

    #-----------------------------
    # Build accept product states
    #-----------------------------
    # TS needs to be built and Büchi accept states
    # defined before calling this function
    def build_accept(self):
        # Reset initial set
        self.graph['accept'] = set()
        # Go through all TS states
        for ts_node in self.graph['ts'].nodes():
            # Go through all accept states in Büchi
            for buchi_accept in self.graph['buchi'].graph['accept']:
                accept_prod_node = (ts_node, buchi_accept)
                # If Büchi state is an accept state, build composed node and add it to the product accept set
                self.graph['accept'].add(accept_prod_node)

    def accept_predecessors(self, accept_node):
        pre_set = set()
        t_ts_node, t_buchi_node = self.projection(accept_node)
        for f_ts_node, cost in self.graph['ts'].fly_predecessors(t_ts_node):
            for f_buchi_node in self.graph['buchi'].predecessors(t_buchi_node):
                f_prod_node = self.composition(f_ts_node, f_buchi_node)
                label = self.graph['ts'].node[f_ts_node]['label']
                truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
                total_weight = cost + self.graph['alpha']*dist
                if truth:
                    pre_set.add(f_prod_node)
                    self.add_edge(f_prod_node, accept_node, weight=total_weight)
        return pre_set

    def fly_successors(self, f_prod_node):
        f_ts_node, f_buchi_node = self.projection(f_prod_node)
        # been visited before, and hasn't changed 
        if ((self.node[f_prod_node]['marker'] == 'visited') and 
            (self.graph['ts'].graph['region'].node[
                self.graph['ts'].node[self.node[f_prod_node]['ts']]['region']]['status'] == 'confirmed')):
            for t_prod_node in self.successors(f_prod_node):
                yield t_prod_node, self.edge[f_prod_node][t_prod_node]['weight']
        else:
            self.remove_edges_from(self.out_edges(f_prod_node))
            for t_ts_node,cost in self.graph['ts'].fly_successors(f_ts_node):
                for t_buchi_node in self.graph['buchi'].successors(f_buchi_node):
                    t_prod_node = self.composition(t_ts_node, t_buchi_node)
                    label = self.graph['ts'].node[f_ts_node]['label']
                    truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
                    total_weight = cost + self.graph['alpha']*dist
                    if truth:
                        self.add_edge(f_prod_node, t_prod_node, weight=total_weight)
                        yield t_prod_node, total_weight
            self.node[f_prod_node]['marker'] = 'visited'

    #------------------------------------
    # Get reachable states from previous
    # reachable set and a given TS state 
    #------------------------------------
    def get_reachable(self, ts_node):
        new_reachable = set()
        # Go through each product state in reachable states
        for f_s in self.reachable_states:
            print "---------- check reachable state ----------"
            print f_s
            # Go through all connected states and if TS states match, add it to the list
            for t_s in self.successors(f_s):
                print "successor is"
                print t_s
                print "and ts node is"
                print ts_node
                if t_s[0] == ts_node:
                    new_reachable.add(t_s)
        return new_reachable

class ProdAut_Run(object):
    # prefix, suffix in product run
    # prefix: init --> accept, suffix accept --> accept
    # line, loop in ts
    def __init__(self, product, prefix, precost, suffix, sufcost, totalcost):
        self.prefix = prefix
        self.precost = precost
        self.suffix = suffix
        self.sufcost = sufcost
        self.totalcost = totalcost
        #self.prod_run_to_prod_edges(product)
        self.plan_output(product)

    def prod_run_to_prod_edges(self):
        self.pre_prod_edges = zip(self.prefix[0:-1], self.prefix[1:])
        self.suf_prod_edges = zip(self.suffix[0:-1], self.suffix[1:])
        #########
        # line: a, b ,c , d, e, g 
        # pre_plan: act_a, act_b, act_c, act_d, act_e, act_g
        # loop: g, b, c, d, e, f, g
        # suf_plan: act_b, act_c, act_d.., act_g
        

    def plan_output(self, product):

        # Collect the nodes of the TS associated with the prefix plan
        self.line = [product.node[node]['ts'] for node in self.prefix]
        # Collect the nodes of the TS associated with the suffix plan
        self.loop = [product.node[node]['ts'] for node in self.suffix]
        # Append start of loop to the end to create a 'loop'
        self.loop.append(self.loop[0])


        # Collect prefix nodes in list of tuples e.g. [ (prefix_node_1, prefix_node_2), (prefix_node_2, prefix_node_3), ..., (prefix_node_n-1, prefix_node_n)]
        self.pre_ts_edges = zip(self.line[0:-1], self.line[1:])
        # Collect suffix nodes in list of tuples (see pre_ts_edges)
        self.suf_ts_edges = zip(self.loop[0:-1], self.loop[1:])

        # output plan --- for execution

        # Initialize prefix plan and cost
        self.pre_plan = list()
        # Initialize pre_plan cost
        self.pre_plan_cost = [0,]

        # Iterate over the nodes associated with the prefix (see pre_ts_edges)
        for ts_edge in self.pre_ts_edges:

            # Extract 'action' label between the two consecutive TS nodes of the prefix plan and add it to the pre_plan
            self.pre_plan.append( product.graph['ts'][ts_edge[0]][ts_edge[1]]['action'] )

            # Add the 'weight' label between the two consectuve TS nodes as the cost of the prefix plan
            self.pre_plan_cost.append(product.graph['ts'][ts_edge[0]][ts_edge[1]]['weight']) # cost 

        # Initialize suffix plan and cost
        self.suf_plan = list()
        self.suf_plan_cost = [0,]

        # Iterate over the nodes associated with the suffix (see suf_ts_edges)
        for ts_edge in self.suf_ts_edges:

            # Extract 'action' label between the two consecutive TS nodes of the suffix plan and add it to the suf_plan
            self.suf_plan.append( product.graph['ts'][ts_edge[0]][ts_edge[1]]['action'] )

            # Add 'weight' label between the consecutive TS nodes of the suffix plan to the cost
            self.suf_plan_cost.append(product.graph['ts'][ts_edge[0]][ts_edge[1]]['weight']) # cost

        print('pre_plan: ' + str(self.pre_plan))
        print('suf_plan: ' + str(self.suf_plan))


    ##OLD plan_output##
    # def plan_output(self, product):

    #     # Collect the nodes of the TS associated with the prefix plan
    #     self.line = [product.node[node]['ts'] for node in self.prefix]
    #     # Collect the nodes of the TS associated with the suffix plan
    #     self.loop = [product.node[node]['ts'] for node in self.suffix]

    #     # Collect prefix nodes in list of tuples e.g. [ (prefix_node_1, prefix_node_2), (prefix_node_2, prefix_node_3), ..., (prefix_node_n-1, prefix_node_n)]
    #     self.pre_ts_edges = zip(self.line[0:-1], self.line[1:])
    #     # Collect suffix nodes in list of tuples (see pre_ts_edges)
    #     self.suf_ts_edges = zip(self.loop[0:-1], self.loop[1:])

    #     # output plan --- for execution

    #     # Initialize prefix plan: Extract first state of the state_model associated with the prefix plan
    #     self.pre_plan = [self.line[0][0],]
    #     # Initialize pre_plan cost
    #     self.pre_plan_cost = [0,]

    #     # Iterate over the nodes associated with the prefix (see pre_ts_edges)
    #     for ts_edge in self.pre_ts_edges:
    #         # Check the action for the consecutive set of nodes in the prefix. If the action is 'goto', add the next motion state (ie region) as the plan's action
    #         if product.graph['ts'][ts_edge[0]][ts_edge[1]]['action'] == 'goto':
    #             self.pre_plan.append(ts_edge[1][0]) # motion NOTE: here the motion is associated with the TS model and stored in order of Motion then Action 
    #         else:
    #             # if the action is NOT 'goto', then we add the 'action' to the pre plan
    #             self.pre_plan.append(ts_edge[1][1]) # action

    #         # add the weight of the weight from the TS between the two consectuve nodes as the cost of the plan
    #         self.pre_plan_cost.append(product.graph['ts'][ts_edge[0]][ts_edge[1]]['weight']) # cost 

    #     # Initialize suffix plan and cost
    #     self.suf_plan = list()
    #     self.suf_plan_cost = [0,]

    #     # Iterate over the nodes associated with the suffix (see suf_ts_edges)
    #     for ts_edge in self.suf_ts_edges:
    #         if product.graph['ts'][ts_edge[0]][ts_edge[1]]['action'] == 'goto':
    #             self.suf_plan.append(ts_edge[1][0]) # motion 
    #         else:
    #             self.suf_plan.append(ts_edge[1][1]) # action
    #         self.suf_plan_cost.append(product.graph['ts'][ts_edge[0]][ts_edge[1]]['weight']) # cost

        









