# -*- coding: utf-8 -*-

from product import ProdAut_Run
from collections import defaultdict
from networkx import dijkstra_predecessor_and_distance, has_path, shortest_simple_paths, shortest_path, all_simple_paths

import time 


#===========================================
#optimal initial synthesis
#===========================================
def dijkstra_plan_networkX(product, beta=10):
	# requires a full construct of product automaton
	start = time.time()
	runs = {}
	loop = {}
	# minimal circles
	for prod_target in product.graph['accept']:
                #print 'prod_target', prod_target
                # accepting state in self-loop
                if prod_target in product.predecessors(prod_target):
                        loop[prod_target] = (product[prod_target][prod_target]["weight"], [prod_target, prod_target])
                        continue
                else:
                        cycle = {}
                        loop_pre, loop_dist = dijkstra_predecessor_and_distance(product, prod_target)
                        for target_pred in product.predecessors(prod_target):
                                if target_pred in loop_dist:
                                        cycle[target_pred] = product[target_pred][prod_target]["weight"] + loop_dist[target_pred]
                        if cycle:
                                opti_pred = min(cycle, key = cycle.get)
                                suffix = compute_path_from_pre(loop_pre, opti_pred)
                                loop[prod_target] = (cycle[opti_pred], suffix)
	# shortest line
	for prod_init in product.graph['initial']:
                line = {}
		line_pre, line_dist = dijkstra_predecessor_and_distance(product, prod_init)
		for target in loop.iterkeys():
			if target in line_dist:
				line[target] = line_dist[target]+beta*loop[target][0]
		if line:
			opti_targ = min(line, key = line.get)
			prefix = compute_path_from_pre(line_pre, opti_targ)
			precost = line_dist[opti_targ]
			runs[(prod_init, opti_targ)] = (prefix, precost, loop[opti_targ][1], loop[opti_targ][0])
	# best combination
	if runs:
		prefix, precost, suffix, sufcost = min(runs.values(), key = lambda p: p[1] + beta*p[3])
		run = ProdAut_Run(product, prefix, precost, suffix, sufcost, precost+beta*sufcost)
		print '=================='
		print 'Dijkstra_plan_networkX done within %.2fs: precost %.2f, sufcost %.2f' %(time.time()-start, precost, sufcost)
		return run, time.time()-start
		#print '\n==================\n'
	print '=================='        
	print 'No accepting run found in optimal planning!'
        return None, None

def compute_path_ac_d(product, path):
        ac_d = 0
        for i in range(len(path)-1):
                e = (path[i], path[i+1])
                ac_d += product[e[0]][e[1]]['distance']
        return ac_d


def opt_path_jit(product, posb_runs):
        opt_line_path = min(posb_runs, key=lambda p: compute_path_ac_d(product, p))
        return opt_line_path

def opt_path_in_prefix(product, ts_path, reachable_states):
        path_pool = []
        for init in product.graph['initial']:
                if init[0] == ts_path[0]:                        
                        for end_state in reachable_states:
                                #for line_path in shortest_simple_paths(product, init, end_state, 'weight'):
                                for line_path in all_simple_paths(product, init, end_state, cutoff=len(ts_path)):
                                        if len(line_path) == len(ts_path):
                                                if all(line_path[k][0] == ts_path[k] for k in range(0, len(ts_path))):
                                                        path_pool.append(line_path)
                                                        #break
        opt_line_path = min(path_pool, key=lambda p: compute_path_ac_d(product, p))
        return opt_line_path

def opt_path_in_suffix(product, ts_path, reachable_states):
        path_pool = []
        for acc in product.graph['accept']:
                if acc[0] == ts_path[0]:
                        for end_state in reachable_states:
                                #for line_path in shortest_simple_paths(product, acc, end_state, 'weight'):
                                for line_path in all_simple_paths(product, acc, end_state, cutoff=len(ts_path)):
                                        if len(line_path) == len(ts_path):
                                                if all(line_path[k][0] == ts_path[k] for k in range(0, len(ts_path))):
                                                        path_pool.append(line_path)
                                                        #break
        opt_line_path = min(path_pool, key=lambda p: compute_path_ac_d(product, p))
        return opt_line_path        
        

def dijkstra_path_networkX(product, start, goal):
        opt_path = shortest_path(product, start, goal, 'weight')
        return opt_path                

        
def dijkstra_plan_optimal(product, beta=10, start_set=None):
	start = time.time()
	#print 'dijkstra plan started!'
	runs = {}
	accept_set = product.graph['accept']
	if start_set == None:
		init_set = product.graph['initial']
	else:
		init_set = start_set
	#print 'number of accepting states %d' %(len(accept_set))
	#print 'number of initial states %d' %(len(init_set))
	loop_dict = {}
	for init_prod_node in init_set:
		for (prefix, precost) in dijkstra_targets(product, init_prod_node, accept_set):
			#print 'accept node reached %s' %(str(prefix[-1]))
			if prefix[-1] in loop_dict:
				suffix, sufcost = loop_dict[prefix[-1]]
			else:
				suffix, sufcost = dijkstra_loop(product, prefix[-1])
				#print suffix, sufcost
				loop_dict[prefix[-1]] = (suffix, sufcost)
			if suffix:
				runs[(prefix[0], prefix[-1])] = (prefix, precost, suffix, sufcost)
				#print 'find run from %s to %s and back' %(str(init_prod_node), str(prefix[-1]))
	if runs:
	 	prefix, precost, suffix, sufcost = min(runs.values(), key = lambda p: p[1] + beta*p[3])
	 	run = ProdAut_Run(product, prefix, precost, suffix, sufcost, precost+beta*sufcost)
	 	#print '\n==================\n'
	 	print 'optimal_dijkstra_olf done within %.2fs: precost %.2f, sufcost %.2f' %(time.time()-start, precost, sufcost)
	 	return run, time.time()-start
	print 'no accepting run found in optimal planning!'


def dijkstra_plan_bounded(product, time_limit=3, beta=10):
	start = time.time()
	print 'dijkstra plan started!'
	runs = {}
	accept_set = product.graph['accept']
	init_set = product.graph['initial']
	print 'number of accepting states %d' %(len(accept_set))
	print 'number of initial states %d' %(len(init_set))
	loop_dict = {}
	for init_prod_node in init_set:
		for (prefix, precost) in dijkstra_targets(product, init_prod_node, accept_set):
			#print 'accept node reached %s' %(str(prefix[-1]))
			if prefix[-1] in loop_dict:
				suffix, sufcost = loop_dict[prefix[-1]]
			else:
				suffix, sufcost = dijkstra_loop(product, prefix[-1])
				loop_dict[prefix[-1]] = (suffix, sufcost)
			#print suffix, sufcost
			if suffix:
				runs[(prefix[0], prefix[-1])] = (prefix, precost, suffix, sufcost)
				#print 'find run from %s to %s and back' %(str(init_prod_node), str(prefix[-1]))
			if time.time()-start > time_limit:  # time limit has reached
				if runs:
				 	prefix, precost, suffix, sufcost = min(runs.values(), key = lambda p: p[1] + beta*p[3])
				 	run = ProdAut_Run(product, prefix, precost, suffix, sufcost, precost+beta*sufcost)
				 	print 'optimal_dijkstra done within %.2fs: precost %.2f, sufcost %.2f' %(time.time()-start, precost, sufcost)
				 	return run, time.time()-start
	print 'no accepting run found in optimal planning!'
	

def dijkstra_targets(product, prod_source, prod_targets):
	# for product graph only, shortest path from source to a set of targets
	tovisit = set()
	visited = set()
	dist  = defaultdict(lambda: float('inf'))
	pre_node = {}
	dist[prod_source] = 0
	tovisit.add(prod_source)
	feasible_targets = set()
	for prod_accep in prod_targets:
		accept_pre_set = product.accept_predecessors(prod_accep)
		if accept_pre_set:
			feasible_targets.add(prod_accep)
	while (tovisit and feasible_targets):
		f_prod_node = min(tovisit, key=lambda n: dist[n])
		tovisit.remove(f_prod_node)
		visited.add(f_prod_node)
		d = dist[f_prod_node]
		for (t_prod_node, cost) in product.fly_successors_iter(f_prod_node):
		 	nd = d + cost 
		 	if nd < dist[t_prod_node]:
		 		dist[t_prod_node] = nd
		 		pre_node[t_prod_node] = [f_prod_node]
		 	if t_prod_node not in visited:
		 		tovisit.add(t_prod_node)
	 	if f_prod_node in feasible_targets:
	 	#print 'found path to buchi_target %s' %str(buchi_target)
	 		feasible_targets.remove(f_prod_node)
	 		yield compute_path_from_pre(pre_node, f_prod_node), dist[f_prod_node]


def dijkstra_loop(product, prod_accep):
	#print 'accept node check %s' %(str(prod_accep))
	paths = {}
	costs = {}
	accept_pre_set = product.accept_predecessors(prod_accep)
	for (tail, cost) in dijkstra_targets(product, prod_accep, accept_pre_set):
		if tail:
			accep_pre = tail[-1]
			paths[accep_pre] = tail
			costs[accep_pre] = cost + product[accep_pre][prod_accep]['weight']
	if costs:
		min_pre = min(costs.keys(), key=lambda p: costs[p])
		min_loop =  paths[min_pre]
		return min_loop, costs[min_pre]
	else:
		return None, None


def compute_path_from_pre(pre, target):
	#print 'pre: %s with size %i' %(pre, len(pre))
	n = target
	path = [n]
	while n in pre:
		#print 'before append'
		#print 'now at node %s' %str(n)
		pn_list = pre[n]
		#print 'its pre_list %s' %str(pn_list)
		if not pn_list:
			break
		pn = pn_list[0]
		#print '[0] of pn_list %s' %str(pn)
		path.append(pn)
		#print 'path: %s' %path
		n = pn
	path.reverse()
	return path

#===========================================
#improve the current plan
#===========================================
def prod_states_given_history(product, trace):
	if trace:
		S1 = set([(trace[0],p) for p in product.graph['buchi'].graph['initial']])
		for p in trace[1:-1]:
			S2 = set()
			for f_node in S1:
				for t_node in product.fly_successors_iter(f_node):
					if t_node[0]==p:
						S2.add(t_node)
			S1 = S2.copy()
		return S1
	else:
		return set()


def improve_plan_given_history(product, trace):
	new_initial_set = prod_states_given_history(product, trace)
	if new_initial_set:
		new_run, time=dijkstra_plan_optimal(product, 10, new_initial_set)
		return new_run
	else:
		return None


#===========================================
#local revision, in case of system update
#===========================================
def validate_and_revise_after_ts_change(run, product, sense_info, com_info):
	new_prefix = None
	new_suffix = None
	start = time.time()
	changed_regs = product.graph['ts'].graph['region'].update_after_region_change(sense_info, com_info)
	if changed_regs:
		for (index, prod_edge) in enumerate(run.pre_prod_edges):
			(f_ts_node, f_buchi_node) = prod_edge[0]
			(t_ts_node, t_buchi_node) = prod_edge[1] 
			succ_prod = set()
			for prod_node_to, weight in product.graph['ts'].fly_successors_iter(f_ts_node):
				succ_prod.add(prod_node_to)
			if t_ts_node not in succ_prod:
					print 'Oops, the current plan prefix contains invalid edges, need revision!'
					new_prefix = dijkstra_revise_once(product, run.prefix, index)
					break
		for (index, prod_edge) in enumerate(run.suf_prod_edges):
			(f_ts_node, f_buchi_node) = prod_edge[0]
			(t_ts_node, t_buchi_node) = prod_edge[1] 
			succ_prod = set()
			for prod_node_to, weight in product.graph['ts'].fly_successors_iter(f_ts_node):
				succ_prod.add(prod_node_to)
			if t_ts_node not in succ_prod:
					print 'Oops, the current plan suffix contains invalid edges, need revision!'
					new_prefix = dijkstra_revise_once(product, run.suffix, index)
					break
		if new_prefix or new_suffix:
			if new_prefix:
				run.prefix = new_prefix
			if new_suffix:
				run.suffix = new_suffix
			run.prod_run_to_prod_edges(product)
			run.output(product)
			print 'validate_and_revise_after_ts_change done in %.2fs' %(time.time()-start)
		else:
			print 'local revision failed'
			return False


def dijkstra_revise(product, run_segment, broken_edge_index):
	suf_segment = run_segment[(broken_edge_index+1):-1]
	for (bridge, cost) in dijkstra_targets(product, run_segment[broken_edge_index-1], suf_segment):
		run_segment_reversed = run_segment
		run_segment_reversed.reverse()
		index = run_segment_reversed.index(bridge[-1])
		index = len(run_segment)-index-1
		new_run_segment = run_segment[0:(broken_edge_index-1)] + bridge + run_segment[(index+1):-1]
		return new_run_segment


def dijkstra_revise_once(product, run_segment, broken_edge_index):
	for (bridge, cost) in dijkstra_targets(product, run_segment[broken_edge_index-1], set([run_segment[-1]])):
		new_run_segment = run_segment[0:(broken_edge_index-1)] + bridge
		return new_run_segment

def has_path_to_accept(product, f_s):
        for acc in product.graph['accept']:
                if has_path(product, f_s, acc):
                        return True
        return False

def compute_path_cost(G, path):
        cost = 0
        for k in range(0,len(path)-1):
                cost += G[path[k]][path[k+1]]['weight']
        return cost
        
def add_temp_task(product, run, index, segment, reg_s, reg_g, t_sg):
        if segment == 'line':
                new_pre = run.prefix[index:] + run.suffix
        elif segment == 'loop':
                new_pre = run.suffix[index:] + run.suffix[0:index]
        new_line = [product.node[node]['ts'] for node in new_pre]
        K = len(new_line)
        ts = product.graph['ts']
        w = 10
        eval = 1000
        for s in range(K-2):
                for g in range(s,K-1):
                        #---------- for r_s
                        opt_path_to_r_s = shortest_path(ts, new_line[s], reg_s, 'weight')                        
                        cost_to_r_s = compute_path_cost(ts, opt_path_to_r_s)
                        opt_path_r_s_back = shortest_path(ts, reg_s, new_line[s+1], 'weight')                        
                        cost_r_s_back = compute_path_cost(ts, opt_path_r_s_back)
                        wo_r_s = ts[new_line[s]][new_line[s+1]]['weight']
                        extra_cost += cost_to_r_s + cost_r_s_back - wo_r_s
                        #---------- for r_g
                        opt_path_to_r_g = shortest_path(ts, new_line[g], reg_g, 'weight')                        
                        cost_to_r_g = compute_path_cost(ts, opt_path_to_r_g)
                        opt_path_r_g_back = shortest_path(ts, reg_g, new_line[g+1], 'weight')                        
                        cost_r_g_back = compute_path_cost(ts, opt_path_r_g_back)
                        wo_r_g = ts[new_line[g]][new_line[g+1]]['weight']
                        extra_cost += cost_to_r_g + cost_r_g_back - wo_r_g
                        #-------- total time
                        total_time = compute_path_cost(ts, new_line[0:(g+1)]) + extra_cost 
                        if total_time > t_sg: # delay
                                delay_time = total_time - t_sg
                        else:
                                delay_time = 0
                        new_eval = w*delay_time + extra_cost
                        if new_eval < eval:
                                eval = new_eval
                                new_s = s
                                new_g = g
        print 'Best index s and g found: (s, g) = (%d, %d)' %(new_s, new_g)
        if (g - s == 1):
                best_line = (new_line[0:s]
                             +  shortest_path(ts, new_line[s], reg_s, 'weight')
                             + shortest_path(ts, reg_s, new_line[s+1], 'weight')[0:-2]
                             + shortest_path(ts, new_line[g], reg_g, 'weight')
                             + shortest_path(ts, reg_g, new_line[g+1], 'weight'))
        else:
                best_line = (new_line[0:s]
                             +  shortest_path(ts, new_line[s], reg_s, 'weight')
                             + shortest_path(ts, reg_s, new_line[s+1], 'weight')
                             + newline[(s+2):g]
                             + shortest_path(ts, new_line[g], reg_g, 'weight')
                             + shortest_path(ts, reg_g, new_line[g+1], 'weight'))
        if (g+2 <= K-1):
                best_line += newline[(g+2):]
        best_precost = compute_path_cost(ts, best_line)
        return best_line, best_precost
        
