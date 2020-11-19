# -*- coding: utf-8 -*-
from buchi import mission_to_buchi
from product import ProdAut
#from ts import distance, reach_waypoint
from discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal, improve_plan_given_history
import matplotlib.pyplot as plt
import networkx as nx

class LTLPlanner(object):
    def __init__(self, ts, hard_spec, soft_spec):
        buchi = mission_to_buchi(hard_spec, soft_spec)
        self.product = ProdAut(ts, buchi)
        self.Time = 0
        self.cur_pose = None
        self.trace = [] # record the regions been visited
        self.traj = [] # record the full trajectory
        self.opt_log = [] 
        # record [(time, prefix, suffix, prefix_cost, suffix_cost, total_cost)]
        self.com_log = []
        # record [(time, no_messages)]

    def optimal(self, beta=10, style='static'):
        self.beta = beta
        if style == 'static':
            # full graph construction
            self.product.graph['ts'].build_full()
            self.product.build_full()
            self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
        elif style == 'ready':
            self.product.build_full()
            self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
        elif style == 'on-the-fly':
            # on-the-fly construction
            self.product.build_initial()
            self.product.build_accept() 
            self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
            if self.run == None:
                print '---No valid plan has been found!---'
                print '---Check you FTS or task---'
                return 
        #print '\n'
        print '------------------------------'
        print 'the prefix of plan **states**:'
        print [n for n in self.run.line]
        print 'the suffix of plan **states**:'
        print [n for n in self.run.loop]
        print '------------------------------'
        print 'the prefix of plan **aps**:'
        print [self.product.graph['ts'].node[n]['label'] for n in self.run.line]
        print 'the suffix of plan **aps**:'
        print [self.product.graph['ts'].node[n]['label'] for n in self.run.loop]
        #print '\n'
        print '------------------------------'
        # print 'the prefix of plan **actions**:'
        # print [n for n in self.run.pre_plan]
        # print 'the suffix of plan **actions**:'
        # print [n for n in self.run.suf_plan]
        self.opt_log.append((self.Time, self.run.pre_plan, self.run.suf_plan, self.run.precost, self.run.sufcost, self.run.totalcost))
        self.last_time = self.Time
        self.acc_change = 0
        self.index = 0
        self.segment = 'line'
        self.next_move = self.run.pre_plan[self.index]
        return plantime

    # Check if given TS state in trap (if reached, no possible path to accept)
    def is_trap(self, ts_state):
        # Get reachable if current reachable were to be updated from a given TS state
        new_reachable = self.product.update_reachable(ts_state)

        # If reachable states exist
        if new_reachable:
            # If TS state is trap
            if self.product.check_reachable_for_trap(new_reachable):
                return 1
            # If TS state is not a trap
            else:
                return -1
        # If no reachables states, TS is not connected to current state
        else:
            return 0

    def find_next_move(self):
        # Check if plan is in 'line' i.e. prefix or 'loop' i.e. suffix

        # if index is not the last of the pre_plan...
        if self.segment == 'line' and self.index < len(self.run.pre_plan)-1:

            # Add the node that has been visited to trace
            self.trace.append(self.run.line[self.index])

            # Increment index counter
            self.index += 1

            # Extract next move from pre_plan
            self.next_move = self.run.pre_plan[self.index]

        # If index is the last of the pre-plan or equivalently if the pre_plan is short...
        elif (self.segment == 'line') and ((self.index == len(self.run.pre_plan)-1) or (len(self.run.pre_plan) <= 1)):

            # Add the node that has been visited to trace
            self.trace.append(self.run.line[self.index])

            # Reset index for the suffix loop
            self.index = 0

            # Change the segment type to loop
            self.segment = 'loop'

            # Extract first move of suffix plan
            self.next_move = self.run.suf_plan[self.index]

        # If index is not the last of the suffix plan or equivalently the suf_plan is short...
        elif self.segment == 'loop' and self.index < len(self.run.suf_plan)-1:

            # Add the node that has been visited to trace
            self.trace.append(self.run.loop[self.index])

            # Increment the index
            self.index += 1

            # Extract next move from suffix plan
            self.next_move = self.run.suf_plan[self.index]

        # If index is the last element of the suf_plan or equivalently the suf_plan is short....
        elif (self.segment == 'loop') and ((self.index == len(self.run.suf_plan)-1) or (len(self.run.suf_plan) <= 1)):

            # Add the node that has been visited to trace
            self.trace.append(self.run.loop[self.index])

            # Resent index 
            self.index = 0

            # Extract next move from suffix plan
            self.next_move = self.run.suf_plan[self.index]
        return self.next_move

    #--------------------------------------
    # Given a new initial TS state, replan
    #--------------------------------------
    def replan_from_ts_state(self, ts_state):
        # Set new initial state in TS
        self.product.graph['ts'].set_initial(ts_state)
        # Use on-the-fly to only rebuild the initial product node
        self.optimal(style="on-the-fly")

    def replan(self):
        '''Create new system plan based on previous history'''
        new_run = improve_plan_given_history(self.product, self.trace)

        print('new_run = ' + str(new_run))
        print '------------------------------'
        print 'the prefix of plan **states**:'
        print [n for n in new_run.line]
        print 'the suffix of plan **states**:'
        print [n for n in new_run.loop]

        if (new_run) and (new_run.pre_plan !=self.run.pre_plan[self.index:-1]):
            self.run = new_run
            self.index = 1
            self.segment = 'line'
            self.next_move = self.run.pre_plan[self.index]
            print 'Plan adapted!'

    # OLD find_next_move
    # def find_next_move(self):
    #     # Check if plan is in 'line' i.e. prefix or 'loop' i.e. suffix
    #     # Also, if the index is 
    #     if self.segment == 'line' and self.index < len(self.run.pre_plan)-2:
    #         self.trace.append(self.run.line[self.index])
    #         self.index += 1
    #         self.next_move = self.run.pre_plan[self.index]
    #     elif (self.segment == 'line') and ((self.index == len(self.run.pre_plan)-2) or (len(self.run.pre_plan) <= 2)):
    #         self.trace.append(self.run.line[self.index])
    #         self.index = 0
    #         self.segment = 'loop'
    #         self.next_move = self.run.suf_plan[self.index]
    #     elif self.segment == 'loop' and self.index < len(self.run.suf_plan)-2:
    #         self.trace.append(self.run.loop[self.index])
    #         self.index += 1
    #         self.segment = 'loop'
    #         self.next_move = self.run.suf_plan[self.index]
    #     elif (self.segment == 'loop') and ((self.index == len(self.run.suf_plan)-2) or (len(self.run.suf_plan) <= 2)):
    #         self.trace.append(self.run.loop[self.index])
    #         self.index = 0
    #         self.segment = 'loop'
    #         self.next_move = self.run.suf_plan[self.index]
    #     return self.next_move


    # def update(self,object_name):
    #     MotionFts = self.product.graph['ts'].graph['region']
    #     cur_region = MotionFts.closest_node(self.cur_pose)
    #     sense_info = dict()
    #     sense_info['label'] = set([(cur_region,set([object_name,]),set()),]) 
    #     changes = MotionFts.update_after_region_change(sense_info,None)
    #     if changes:
    #         return True

    # def replan(self):
    #     new_run = improve_plan_given_history(self.product, self.trace)
    #     if (new_run) and (new_run.pre_plan !=self.run.pre_plan[self.index:-1]):
    #         self.run = new_run
    #         self.index = 1
    #         self.segment = 'line'
    #         self.next_move = self.run.pre_plan[self.index]
    #         print 'Plan adapted!'











