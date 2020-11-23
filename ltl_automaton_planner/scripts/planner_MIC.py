# -*- coding: utf-8 -*-
from buchi import mission_to_buchi
from product import ProdAut, ProdAut_Run
from ts import distance, reach_waypoint
from discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal, improve_plan_given_history, has_path_to_accept, dijkstra_path_networkX, opt_path_in_prefix, opt_path_in_suffix, opt_path_jit

import time


class ltl_planner(object):
	def __init__(self, ts, hard_spec, soft_spec, beta):
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
                self.beta = beta

        def reset_beta(self, beta):
                self.beta = beta
                self.product.graph['beta'] = beta

        def set_to_suffix(self):
                self.segment = 'loop'
                self.index = 0
                self.next_move = self.run.suf_plan[self.index]
                
        def start_suffix(self):
                if ((self.segment == 'loop') and (self.index == 0)):
                        return True
                else:
                        return False

        def in_suffix(self):
                if ((self.segment == 'loop')):
                        return True
                else:
                        return False                        
                
	def optimal(self, gamma=10, style='static'):
		self.gamma = gamma
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
			self.run, plantime = dijkstra_plan_optimal(self.product, self.gamma)
                elif style == 'repeat':
                        self.run, plantime = dijkstra_plan_networkX(self.product, self.gamma)
                if self.run == None:
                        print '---No valid has been found!---'
                        print '---Check you FTS or task---'
                        return 
		#print '\n'
                # print '------------------------------'
                # print 'the prefix of plan **states**:'
		# print [n for n in self.run.line]
                # print 'the suffix of plan **states**:'
		# print [n for n in self.run.loop]
		# #print '\n'
                # print '------------------------------'
		# print 'the prefix of plan **actions**:'
		# print [n for n in self.run.pre_plan]
		# print 'the suffix of plan **actions**:'
		# print [n for n in self.run.suf_plan]
		# self.opt_log.append((self.Time, self.run.pre_plan, self.run.suf_plan, self.run.precost, self.run.sufcost, self.run.totalcost))
		self.last_time = self.Time
		self.acc_change = 0
		self.index = 0
		self.segment = 'line'
		self.next_move = self.run.pre_plan[self.index]
		return plantime

	def find_next_move(self):
		if self.segment == 'line' and self.index < len(self.run.pre_plan)-2:
			self.trace.append(self.run.line[self.index])
			self.index += 1
			self.next_move = self.run.pre_plan[self.index]
		elif (self.segment == 'line') and ((self.index == len(self.run.pre_plan)-2) or (len(self.run.pre_plan) <= 2)):
			self.trace.append(self.run.line[self.index])
			self.index = 0
			self.segment = 'loop'
			self.next_move = self.run.suf_plan[self.index]
		elif self.segment == 'loop' and self.index < len(self.run.suf_plan)-2:
			self.trace.append(self.run.loop[self.index])
			self.index += 1
			self.segment = 'loop'
			self.next_move = self.run.suf_plan[self.index]
		elif (self.segment == 'loop') and ((self.index == len(self.run.suf_plan)-2) or (len(self.run.suf_plan) <= 2)):
			self.trace.append(self.run.loop[self.index])
			self.index = 0
			self.segment = 'loop'
			self.next_move = self.run.suf_plan[self.index]
		return self.next_move

        def reach_ts_node(self, pose, reach_bound):
                for n in self.product.graph['ts'].nodes():
                        if ((reach_waypoint(n[0], pose, reach_bound))
                            and (n[1] == 'None')):
                                return n
                return None
                        
        def update_reachable(self, reachable_states, ts_node):
                new_reachable = set()                
                for f_s in reachable_states:
                        for t_s in self.product.successors(f_s):
                                if t_s[0] == ts_node:
                                        new_reachable.add(t_s)
                return new_reachable

        def update_runs(self, prev_runs, ts_node):
                new_runs = set()
                for run in prev_runs:
                        f_s = run[-1]
                        for t_s in self.product.successors(f_s):
                                if t_s[0] == ts_node:
                                        new_run = list(run)
                                        new_run.append(t_s)
                                        new_runs.add(tuple(new_run))
                return new_runs                                        
                

        def prod_dist_to_trap(self, pose, reachable_set):
                mini_dist_reg = min(self.product.graph['ts'].graph['region'].nodes(),
                                   key = lambda s: distance(pose, s))
                mini_dist = distance(mini_dist_reg, pose)
                new_reachable = self.update_reachable(reachable_set, (mini_dist_reg, 'None'))
                if self.check_trap(new_reachable):
                        return mini_dist
                else:
                        return -1

        def check_trap(self, reachable_set):
                for s in reachable_set:
                        if has_path_to_accept(self.product, s):
                                return False
                return True

        def check_accept(self, reachable_set):
                accept_set = self.product.graph['accept']
                if accept_set.intersection(reachable_set):
                        return True
                else:
                        return False

        def intersect_accept(self, reachable_set, reach_ts):
                accept_set = self.product.graph['accept']
                inter_set = set([s for s in accept_set if s[0] == reach_ts])
                return inter_set


	def update(self,object_name):
		MotionFts = self.product.graph['ts'].graph['region']
		cur_region = MotionFts.closest_node(self.cur_pose)
		sense_info = dict()
		sense_info['label'] = set([(cur_region,set([object_name,]),set()),]) 
		changes = MotionFts.update_after_region_change(sense_info,None)
		if changes:
			return True                        

	def replan(self):
		new_run = improve_plan_given_history(self.product, self.trace)
		if (new_run) and (new_run.pre_plan !=self.run.pre_plan[self.index:-1]):
			self.run = new_run
			self.index = 1
			self.segment = 'line'
			self.next_move = self.run.pre_plan[self.index]
			print 'Plan adapted!'

        def compute_path_cost(self, path):
                ac_c = 0
                ac_d = 0
                for i in range(len(path)-1):
                        e = (path[i], path[i+1])
                        ac_d += self.product.edge[e[0]][e[1]]['distance']
                        ac_c += self.product.edge[e[0]][e[1]]['cost']
                return [ac_c, ac_d]

        def margin_opt_path(self, opt_path, beta):
            self.reset_beta(beta)
            self.product.build_full_margin(opt_path)
            #marg_path = dijkstra_path_networkX(self.product, opt_path[0], opt_path[-1])
            self.run, plantime = dijkstra_plan_networkX(self.product, self.gamma)
            return self.run.suffix

                
        def opt_path_match(self, path1, path2):
            score = 0
            for i,s in enumerate(path1):
                if ((i< len(path2)) and (path2[i] == s)):
                    score += 1
            return score

        def find_opt_path(self, ts_path, reachable_prod_states):
                if self.segment == 'line':
                        print 'In prefix'
                        opt_path = opt_path_in_prefix(self.product, ts_path, reachable_prod_states)
                        return opt_path
                elif self.segment == 'loop':
                        print 'In suffix'
                        opt_path = opt_path_in_suffix(self.product, ts_path, reachable_prod_states)
                        return opt_path

        def find_opt_paths_jit(self, posb_runs):
                opt_path = opt_path_jit(self.product, posb_runs)
                return opt_path
                
        def irl_jit(self, posb_runs):
                print '------------------------------'
                print 'Find beta via IRL starts'
                t0 = time.time()
                opt_path = self.find_opt_paths_jit(posb_runs)
                opt_cost = self.compute_path_cost(opt_path)
                opt_ac_d = opt_cost[1]
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
                        #gradient = beta + lam*(opt_ac_d-marg_ac_d)
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
                print 'Find beta via IRL done, time %.2f' %(time.time()-t0)
                print 'In total **%d** para_dijkstra run ||| beta sequence: %s' %(count, str(beta_seq))
                print 'Opt_path length: %d, match score sequence: %s' %(len(opt_path), str(match_score))
                print '--------------------'
                if beta <0:
                        beta = 0
                self.reset_beta(beta)
                self.optimal(style='ready')
                opt_suffix = list(self.run.suffix)
                self.set_to_suffix()
                print 'opt_suffix updated to %s' %str(opt_suffix)
                print '-----------------'
                return beta_seq, match_score                        
            
        def irl(self, ts_path, reachable_prod_states):
                print '------------------------------'
                print 'Find beta via IRL starts'
                t0 = time.time()
                opt_path = self.find_opt_path(ts_path, reachable_prod_states)
                opt_cost = self.compute_path_cost(opt_path)
                opt_ac_d = opt_cost[1]
                beta_seq = [] 
                beta = 100.0
                beta_p = 0.0
                count = 0
                lam = 1.0
                alpha = 1.0
                match_score = []
                count = 0
                while ((abs(beta_p-beta)>0.3) or (count <20)):
                        print 'Iteration --%d--'%count
                        beta = beta_p
                        marg_path = self.margin_opt_path(opt_path, beta)
                        marg_cost = self.compute_path_cost(marg_path)
                        marg_ac_d = marg_cost[1]
                        print '(opt_ac_d-marg_ac_d)', opt_ac_d-marg_ac_d
                        #gradient = beta + lam*(opt_ac_d-marg_ac_d)
                        gradient = lam*(opt_ac_d-marg_ac_d)
                        beta_p = beta - (alpha/(count+1))*gradient
                        print 'gradient:%.2f and beta_dif:%.2f' %(gradient, beta-beta_p)
                        count += 1
                        print 'old beta: %.2f ||| new beta: %.2f' %(beta, beta_p)
                        score = self.opt_path_match(opt_path, marg_path)
                        beta_seq.append(beta_p)
                        match_score.append(score)
                print '--------------------'
                print 'Find beta via IRL done, time %.2f' %(time.time()-t0)
                print 'In total **%d** para_dijkstra run ||| beta sequence: %s' %(count, str(beta_seq))
                print 'Opt_path length: %d, match score sequence: %s' %(len(opt_path), str(match_score))
                print '--------------------'
                self.reset_beta(beta)
                self.optimal(style='ready')
                opt_suffix = list(self.run.suffix)
                self.set_to_suffix()
                print 'opt_suffix updated to %s' %str(opt_suffix)
                print '-----------------'
                return beta_seq, match_score

        def add_temp_task(self, temp_task):
                reg_s = (temp_task[0],temp_task[1])
                reg_g = (temp_task[2],temp_task[3])
                t_sg = temp_task[4]
                best_line, best_precost = add_temp_task(self.product, self.run, self.index, self.segment, reg_s, reg_g, t_sg)
                self.run.update_line(best_line, best_precost)                
                print 'Temporary task Incorporated in plan! new_pre_plan:%s' %str(self.run.pre_plan)
                self.index = 0
		self.segment = 'line'
		self.next_move = self.run.pre_plan[self.index]
                print 'Index reset and start new_line execution'

                







