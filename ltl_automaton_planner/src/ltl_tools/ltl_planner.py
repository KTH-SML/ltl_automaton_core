# -*- coding: utf-8 -*-
from buchi import mission_to_buchi
from product import ProdAut
#from ts import distance, reach_waypoint
from discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal, improve_plan_given_history, has_path_to_accept, has_path_to_accept_with_cycle, select_least_violating_run
import matplotlib.pyplot as plt
import networkx as nx

class LTLPlanner(object):
    def __init__(self, ts, hard_spec, soft_spec, beta=1000, gamma=10):
        buchi = mission_to_buchi(hard_spec, soft_spec)
        self.product = ProdAut(ts, buchi, beta)
        self.Time = 0
        self.cur_pose = None
        self.trace = [] # record the regions been visited
        self.traj = [] # record the full trajectory
        self.opt_log = [] 
        # record [(time, prefix, suffix, prefix_cost, suffix_cost, total_cost)]
        self.com_log = []
        # record [(time, no_messages)]
        self.beta = beta                    # importance of taking soft task into account
        self.gamma = gamma                  # cost ratio between prefix and suffix


    def optimal(self, style='static'):
        
        if style == 'static':
            # full graph construction
            self.product.graph['ts'].build_full()
            self.product.build_full()
            self.run, plantime = dijkstra_plan_networkX(self.product, self.gamma)
        elif style == 'ready':
            self.product.build_full()
            self.run, plantime = dijkstra_plan_networkX(self.product, self.gamma)
        elif style == 'on-the-fly':
            # on-the-fly construction
            self.product.build_initial()
            self.product.build_accept() 
            self.run, plantime = dijkstra_plan_networkX(self.product, self.gamma)
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

    #-----------------------------
    #   Update current possible   
    # states using given TS state 
    #-----------------------------
    def update_possible_states(self, ts_node):
        # Get possible states
        self.product.possible_states = self.product.get_possible_states(ts_node)

        if self.start_suffix():
            print '=============================='
            print '--- New suffix execution---'
            print '=============================='               
            self.product.possible_states = self.intersect_accept(self.product.possible_states, ts_node)

        # If possible states set in not empty, return true
        if self.product.possible_states:
            return True
        # If no states in possible states set, return false
        else:
            return False

    # updates set of possible runs
    def update_posb_runs(self, prev_runs, ts_node):
        new_runs = set()
        for run in prev_runs:
            f_s = run[-1]
            for t_s in self.product.successors(f_s):
                if t_s[0] == ts_node:
                    new_run = list(run)
                    new_run.append(t_s)
                    new_runs.add(tuple(new_run))

        return new_runs

    #---------------------------------
    # Check if given TS state in trap 
    #---------------------------------
    # if reached, no possible path to accept
    def is_trap(self, ts_state):
        # Get possible states if current states were to be updated from a given TS state
        reachable_states = self.product.get_possible_states(ts_state)
        print "===== possible states if trap reached ====="
        print reachable_states
        print "====================================="

        # If reachable states exist
        if reachable_states:
            # If TS state is trap
            if self.check_possible_states_for_trap(reachable_states):
                return 1
            # If TS state is not a trap
            else:
                return -1
        # If no reachables states, TS is not connected to current state
        else:
            return 0

    #--------------------------------------------------------------
    # Check if a possible state set has a path to accepting states
    #--------------------------------------------------------------
    # Returns True if possible state set is from a trap state
    # (meaning there are no path to accepting from trap possible state set)
    def check_possible_states_for_trap(self, possible_state_set):
        for s in possible_state_set:
            print "---- checking state for path to accept with cycle ----"
            print s
            print "-----"
            if has_path_to_accept_with_cycle(self.product, s):
                print "has path"
                print "---------------------------------"
                return False
            else:
                print "has no path"
                print "---------------------------------"
                
        return True

    def intersect_accept(self, possible_states, reach_ts):
        accept_set = self.product.graph['accept']
        inter_set = set([s for s in accept_set if s[0] == reach_ts])
        return inter_set


    def start_suffix(self):
        print "=================="
        print "START SUFFIX TEST"
        print "=================="
        if ((self.segment == 'loop') and (self.index == 0)):
            print "RETURN TRUE"
            return True
        else:
            print "RETURN FALSE"
            return False

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


    #------------------------------------------------------------
    # Functions related to Inverse Reinforcement Learning (IRL)
    #-------------------------------------- ---------------------

    # Selects the run which violates the soft task the least
    def select_least_violating_run(self, posb_runs):
        return select_least_violating_run(self.product, posb_runs)

    # Computes the cost of a path(run) in the product automaton
    def compute_path_cost(self, path):
        ac_c = 0
        ac_d = 0
        for i in range(len(path)-1):
            print self.product[path[i]][path[i+1]]['soft_task_dist']
            ac_d += self.product[path[i]][path[i+1]]['soft_task_dist']
            ac_c += self.product[path[i]][path[i+1]]['transition_cost']
        return [ac_c, ac_d]

    # Sets given value of beta parameter (importance of soft task)
    def set_beta(self, beta):
        self.beta = beta
        self.product.graph['beta'] = beta

    # marginal path (TODO check exactly how this works)
    def margin_opt_path(self, opt_path, beta):
        self.set_beta(beta)
        self.product.build_full_margin(opt_path)
        #marg_path = dijkstra_path_networkX(self.product, opt_path[0], opt_path[-1])
        self.run, plantime = dijkstra_plan_networkX(self.product, self.gamma)
        return self.run.suffix

    # returns score about how well two paths match
    def opt_path_match(self, path1, path2):
        score = 0
        for i,s in enumerate(path1):
            if ((i< len(path2)) and (path2[i] == s)):
                score += 1
        return score

    def irl_jit(self, posb_runs):
        print '------------------------------'
        print 'Find beta via IRL starts'
        opt_path = self.select_least_violating_run(posb_runs)
        opt_cost = self.compute_path_cost(opt_path)
        opt_ac_d = opt_cost[1]
        print opt_ac_d
        beta_seq = [] 
        beta = 100.0
        beta_p = self.beta
        count = 0
        lam = 1.0
        alpha = 1.0
        match_score = []
        count = 0
        while ((abs(beta_p-beta)>0.3) and (count <20)):
            if beta_p < 0:
                break
            print 'Iteration --%d--'%count
            beta = beta_p
            marg_path = self.margin_opt_path(opt_path, beta)
            marg_cost = self.compute_path_cost(marg_path)
            marg_ac_d = marg_cost[1]
            print '(opt_ac_d-marg_ac_d)', opt_ac_d-marg_ac_d

            gradient = lam*(opt_ac_d-marg_ac_d)
            if count <10:
                beta_p = beta - (alpha)*gradient
            else:
                beta_p = beta - (alpha/(count+1))*gradient
            print 'gradient:%.2f and beta_dif:%.2f' %(gradient, beta-beta_p)
            count += 1
            print 'old beta: %.2f ||| new beta: %.2f' %(beta, beta_p)
            score = self.opt_path_match(opt_path, marg_path)
            beta_seq.append(beta_p)
            match_score.append(score)
        print '--------------------'
        print 'In total **%d** para_dijkstra run ||| beta sequence: %s' %(count, str(beta_seq))
        print 'Opt_path length: %d, match score sequence: %s' %(len(opt_path), str(match_score))
        print '--------------------'
        if beta <0:
            beta = 0
        self.set_beta(beta)
        self.optimal(style='ready')
        opt_suffix = list(self.run.suffix)
        print 'opt_suffix updated to %s' %str(opt_suffix)
        print '-----------------'
        return beta_seq, match_score   








