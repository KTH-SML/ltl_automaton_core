# -*- coding: utf-8 -*-

from boolean_formulas.parser import parse as parse_guard

from math import sqrt
from networkx.classes.digraph import DiGraph

def distance(pose1, pose2):
    return (sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001)

def reach_waypoint(pose, waypoint, margin):
    if distance(pose, waypoint)<=margin:
        return True
    else:
        return False

class MotionStateModel(DiGraph):
    def __init__(self):
        DiGraph.__init__(self, initial=set())
        #simple action TS with 3 regions
        self.add_node('r1')
        self.add_node('r2')
        self.add_node('r3')
        self.add_edge('r1','r2', weight=10, action='goto_r2')
        self.add_edge('r2','r1', weight=10, action='goto_r1')
        self.add_edge('r1','r3', weight=10, action='goto_r3')
        self.add_edge('r3','r1', weight=10, action='goto_r1')
        self.add_edge('r3','r2', weight=10, action='goto_r2')
        self.add_edge('r2','r3', weight=10, action='goto_r3')

class LoadStateModel(DiGraph):
    def __init__(self):
        DiGraph.__init__(self, initial=set())
        #simple action TS with 3 regions
        self.add_node('unloaded')
        self.add_node('loaded')
        self.add_edge('unloaded','loaded', weight=50, action='pick')
        self.add_edge('loaded','unloaded', weight=10, action='drop')


class TSModel(DiGraph):
    #take as input a list of state models to combine
    def __init__(self, state_models):
        self.build_full(state_models)

    def build_full(self, state_models):
        # Initialize TS model on first model from state_models
        self.product = state_models[0]

        # If more than one state_model, create a product graph between all models
        if (len(state_models) > 1):
            for sm in state_models[1:]:
                self.product = GraphProduct(self.product, sm)


class GraphProduct(DiGraph):
    def __init__(self, model_a, model_b):
        DiGraph.__init__(self)
        self.model_a = model_a
        self.model_b = model_b
        self.do_product()

    def composition(self, state_a, state_b):
        prod_node = (state_a, state_b)
        if not self.has_node(prod_node):
            #new_label = self.model_a.node[state_a]['label'].union(self.model_b.node[state_b]['label'])
            #==================================
            #  WHAT WEIGHT FOR THE NEW NODE???
            #       To do in edges
            #==================================
            #self.add_node(prod_node, label=new_label, marker='unvisited')
            self.add_node(prod_node, marker='unvisited')
            #===================================
            # TODO: HANDLE INITIAL STATES
            #===================================
            # if ((reg in self.graph['region'].graph['initial']) and (act == 'None')):
            #    self.graph['initial'].add(prod_node)
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


class MotionFts(DiGraph):
    def __init__(self, node_dict, symbols, ts_type):
        DiGraph.__init__(self, symbols=symbols, type=ts_type, initial=set())
        for (n, label) in node_dict.iteritems():
            self.add_node(n, label=label, status='confirmed')
            #print "Adding node ["+str(n)+"[ with label ["+str(label)+"]"
            

    def add_un_edges(self, edge_list, unit_cost=1):
        for edge in edge_list:
            f_node = edge[0]
            t_node = edge[1]
            dist = distance(f_node, t_node)
            self.add_edge(f_node, t_node, weight=dist*unit_cost)
            self.add_edge(t_node, f_node, weight=dist*unit_cost)
        for node in self.nodes():
            #self.add_edge(node, node, weight=unit_cost)
            # allow self-transit to 0-cost
            self.add_edge(node, node, weight=0)

    def add_un_edges_by_ap(self, edge_list, unit_cost=1):
        for edge in edge_list:            
            f_ap = edge[0]
            t_ap = edge[1]
            f_nodes = [n for n in self.nodes() if f_ap in self.node[n]['label']]
            if len(f_nodes)> 1:
                print 'ambiguity more than one with f_ap %s, see %s' %(f_ap, str(f_nodes))
            else:
                f_node = f_nodes[0]
            t_nodes = [n for n in self.nodes() if t_ap in self.node[n]['label']]
            if len(t_nodes)> 1:
                print 'ambiguity more than one with t_ap %s, see %s' %(t_ap, str(t_nodes))
            else:
                t_node = t_nodes[0]
            dist = distance(f_node, t_node)
            self.add_edge(f_node, t_node, weight=dist*unit_cost)
            self.add_edge(t_node, f_node, weight=dist*unit_cost)
        for node in self.nodes():
            #self.add_edge(node, node, weight=unit_cost)
            # allow self-transit to 0-cost
            self.add_edge(node, node, weight=0)            

    def add_full_edges(self,unit_cost=1):
        for f_node in self.nodes():
            for t_node in self.nodes():
                dist = distance(f_node, t_node)
                if (f_node, t_node) not in self.edges():
                    self.add_edge(f_node, t_node, weight=dist*unit_cost)

    def set_initial(self, pose):
        init_node = self.closest_node(pose)
        self.graph['initial'] = set([init_node])
        return init_node

    def closest_node(self, pose):
        node = min(self.nodes(), key= lambda n: distance(n,pose))
        return node

    def update_after_region_change(self, sense_info, com_info, margin=10):
        # sense_info = {'label':set((x,y), l', l'_)), 'edge':(set(add_edges), set(del_edges))}
        # com_info = set((x,y), l', l'_))
        # margin for adding new nodes, NOTE units!
        changed_regs = set()
        # label udpate
        label_info = sense_info['label']
        label_info.update(com_info)
        for mes in label_info:
            if mes[1]:
                close_node = self.closest_node(mes[0])
                if distance(close_node, mes[0])>margin:
                    self.add_node(mes[0], mes[1])
                else:
                    old_label = self.node[close_node]['label']
                    new_label = old_label.union(mes[1]).difference(mes[2])
                    if old_label != new_label:
                        self.node[close_node]['label'] = set(new_label)
                        self.node[close_node]['status'] = 'notconfirmed'
                        changed_regs.add(close_node)
        # edges udpate
        edge_info = sense_info['edge']
        for e in edge_info[0]:
            self.add_edge(e[0], e[1], weight=distance(e[0], e[1]))
            self.node[close_node]['status'] = 'notconfirmed'
            changed_regs.add(e[0])
        for e in edge_info[1]:
            self.remove_edge(e[0], e[1])
            changed_regs.add(e[0])
            self.node[close_node]['status'] = 'notconfirmed'
        return chnaged_regs

# class ActionModel(object):
#     # action_dict = {act_name: (cost, guard_formula, label)}
#     def __init__(self, action_dict):
#         self.raw = action_dict
#         self.action = dict()
#         for act_name, attrib in action_dict.iteritems():
#             cost = attrib[0]
#             guard_formula = attrib[1]
#             guard_expr = parse_guard(guard_formula)
#             label = attrib[2]
#             self.action[act_name] = (cost, guard_expr, label)
#         self.action['None'] = (0, parse_guard('1'), set()) 

#     def allowed_actions(self, ts_node_label):
#         #create empty set
#         allow_action = set()
#         # Iterate over every action in the action model dict
#         for act_name, attrib in self.action.iteritems():
#             #if action guard formula returns true to the region label, add action to the list of allowed action
#             if (attrib[1].check(ts_node_label)):
#                 allow_action.add(act_name)
#         return allow_action

# # =====================
# #  Alternate act model
# # =====================
# class ActionModel2(object):
#     # action_dict = {act_name: (cost, guard_formula, label)}
#     def __init__(self, action_dict):
#         self.raw = action_dict
#         self.action = dict()
#         for act_name, attrib in action_dict.iteritems():
#             cost = attrib[0]
#             guard_formula = attrib[1]
#             guard_expr = parse_guard(guard_formula)
#             label = attrib[2]
#             self.action[act_name] = (cost, guard_expr, label)
#         self.action['None'] = (0, parse_guard('1'), set()) 

#     def allowed_actions(self, ts_node_label):
#         allow_action = set()
#         for act_name, attrib in self.action.iteritems():
#             if (attrib[1].check(ts_node_label)):
#                 allow_action.add(act_name)
#         return allow_action

class CombinedActModel(DiGraph):
    def __init__(self, act_model1, act_model2):
        DiGraph.__init__(self, action_list1=act_model1, action_list2=act_model2, initial=set(), type='ĆombinedActModel')   

    def composition(self, act1, act2):
        comb_node = (act1, act2)
        if not self.has_node(comb_node):
            new_label = self.graph['action_list1'].action[act][2].union(self.graph['action_list2'].action[act][2])
            print "Composing new node:"
            print new_label
            print "-----"
            self.add_node(prod_node, label=new_label, action1=act1, action2=act2, marker='unvisited')
            #if ((reg in self.graph['region'].graph['initial']) and (act == 'None')):
            #    self.graph['initial'].add(prod_node)
        return prod_node

    def build_full(self):
        for list1_act in self.graph['action_list1'].action.iterkeys():
            for list2_act in self.graph['action_list2'].action.iterkeys():
                prod_node = self.composition(list1_act, list2_act)
                # # actions
                if (act == 'None'):
                    label = self.graph['region'].node[reg]['label']
                    for act_to in self.graph['action'].allowed_actions(label):
                        prod_node_to = self.composition(reg, act_to)
                        if act_to != 'None':
                            self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label= act_to, marker= 'visited')
                        else:
                            self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label= 'goto', marker= 'visited')
                # motions
                for reg_to in self.graph['region'].successors(reg):
                    prod_node_to = self.composition(reg_to, 'None')
                    self.add_edge(prod_node, prod_node_to, weight=self.graph['region'][reg][reg_to]['weight'], label= 'goto', marker= 'visited')
        print 'full TS model constructed with %d states and %s transitions' %(len(self.nodes()), len(self.edges())) 


class MotActModel(DiGraph):
    def __init__(self, mot_fts, act_model):
        DiGraph.__init__(self, region=mot_fts, action=act_model, initial=set(), type='MotActModel')

    def composition(self, reg, act):
        prod_node = (reg, act)
        if not self.has_node(prod_node):
            new_label = self.graph['region'].node[reg]['label'].union(self.graph['action'].action[act][2])
            self.add_node(prod_node, label=new_label, region=reg, action=act, marker='unvisited')
            if ((reg in self.graph['region'].graph['initial']) and (act == 'None')):
                self.graph['initial'].add(prod_node)
        return prod_node

    def projection(self, prod_node):
        reg = self.node[prod_node]['region']
        act = self.node[prod_node]['action']
        return reg, act

    def build_initial(self):
        for reg_init in self.graph['region'].graph['initial']:
            init_prod_node = self.composition(reg_init, 'None')

    def build_full(self):
        for reg in self.graph['region'].nodes():
            for act in self.graph['action'].action.iterkeys():
                prod_node = self.composition(reg, act)
                # # actions
                #if no defined action
                if (act == 'None'):
                    #Label is taken from region label (position)
                    label = self.graph['region'].node[reg]['label']
                    #Check every action possible at this region (using label from region)
                    for act_to in self.graph['action'].allowed_actions(label):
                        #Compose a node from the region and each allowed action
                        prod_node_to = self.composition(reg, act_to)
                        #If allowed action is not empty, add an edge between the 'None' node and allowed action node, using action weight and action as edge label
                        if act_to != 'None':
                            self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label= act_to, marker= 'visited')
                        #If action is 'None' as well, add an edge as a goto action
                        else:
                            self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label= 'goto', marker= 'visited')
                # motions
                #If action is not empty, check for every successor regions
                for reg_to in self.graph['region'].successors(reg):
                    #Combine node with successor region without action
                    prod_node_to = self.composition(reg_to, 'None')
                    self.add_edge(prod_node, prod_node_to, weight=self.graph['region'][reg][reg_to]['weight'], label= 'goto', marker= 'visited')
        print 'full TS model constructed with %d states and %s transitions' %(len(self.nodes()), len(self.edges())) 
    
    def fly_successors(self, prod_node): 
        reg, act = self.projection(prod_node)
        # been visited before, and hasn't changed 
        if ((self.node[prod_node]['marker'] == 'visited') and 
            (self.graph['region'].node[self.node[prod_node]['region']]['status'] == 'confirmed')):
            for prod_node_to in self.successors(prod_node):
                yield prod_node_to, self.edges[prod_node, prod_node_to]['weight']
        else:
            self.remove_edges_from(self.out_edges(prod_node))
            # actions 
            label = self.graph['region'].node[reg]['label']
            for act_to in self.graph['action'].allowed_actions(label):
                prod_node_to = self.composition(reg, act_to)
                cost = self.graph['action'].action[act_to][0]
                self.add_edge(prod_node, prod_node_to, weight=cost, label= act_to)
                yield prod_node_to, cost
            # motions
            for reg_to in self.graph['region'].successors(reg):
                if reg_to != reg:
                    prod_node_to = self.composition(reg_to, 'None')
                    cost = self.graph['region'][reg][reg_to]['weight']
                    self.add_edge(prod_node, prod_node_to, weight=cost, label= 'goto')
                    yield prod_node_to, cost
            self.graph['region'].node[self.node[prod_node]['region']]['status'] = 'confirmed'
            self.node[prod_node]['marker'] = 'visited'
        print 'full FTS model constructed with %d states and %s transitions' %(len(self.nodes()), len(self.edges()))             

    def fly_predecessors(self, prod_node): 
        reg, act = self.projection(prod_node)
        # actions
        label = self.graph['region'].node[reg]['label']
        if act in self.graph['action'].allowed_actions(label):    
            for f_act in self.graph['action'].action.iterkeys():
                f_prod_node = self.composition(reg, f_act)
                cost = self.graph['action'].action[act][0]
                self.add_edge(f_prod_node, prod_node, weight=cost, label= act)
                yield f_prod_node, cost
        # motions
        if act == 'None':
            for f_reg in self.graph['region'].predecessors_iter(reg):
                if f_reg !=reg:
                    for f_act in self.graph['action'].action.iterkeys():
                            f_prod_node = self.composition(f_reg, f_act)
                            cost = self.graph['region'][f_reg][reg]['weight']
                            self.add_edge(f_prod_node, prod_node, weight=cost, label= 'goto')         
                            yield f_prod_node, cost
