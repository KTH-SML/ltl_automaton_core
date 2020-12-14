#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from ltl_automaton_planner.ltl_tools.discrete_plan import dijkstra_plan_networkX

#===================================================
#       Inverse Reinforcement Learning Plugin
#===================================================
# As long as the learning trigger is on, the plugin 
# will save the possible runs. Learning is triggered 
# when the trigger  variable goes back to false or 
# when the maximum size of possible runs is reached
#===================================================
class IRLPlugin(object):
    def __init__(self, ltl_planner, args_dict=dict()):
        print "========= CREATED PLUGIN =========="
        print "argument dict is"
        print args_dict
        self.ltl_planner = ltl_planner
        self.beta = self.ltl_planner.product.graph['beta']

        # Maximum size of possible run set (in states)
        try:
            if args_dict:
                self.max_run_buffer_size = args_dict['max_run_buffer_size']
            else:
                self.max_run_buffer_size = 100
                rospy.logwarn("IRL plugin: no maximum buffer size defined, using default value %i" % (self.max_run_buffer_size))
        except KeyError:
            self.max_run_buffer_size = 100
            rospy.logwarn("IRL plugin: no maximum buffer size defined, using default value %i" % (self.max_run_buffer_size))

        # initialize storage of set of possible runs in product
        self.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])

        # Init learning trigger variable
        self.learning_trigger = False 
        self.prev_learning_trigger = False

    def init(self):
        None

    #------------------------------
    # Setup publishers subscribers
    #------------------------------
    def set_sub_and_pub(self):
        # Learning trigger subscriber
        self.learning_trigger_sub = rospy.Subscriber("irl_trigger", Bool, self.learning_trigger_callback)

    #---------------------------------------------------
    # Set learning trigger on or off according to topic
    #---------------------------------------------------
    def learning_trigger_callback(self, msg):
        self.learning_trigger = msg.data

    #------------------------------
    # Run at every TS state update
    #------------------------------
    def run_at_ts_update(self, ts_state):
        #-----------------------------------------------
        # If learning is triggered, update possible runs
        #------------------------------------------------
        if self.learning_trigger:
            # Update set of possible runs
            self.posb_runs = self.update_posb_runs(self.posb_runs, ts_state)

            # Calculate size of possible run (in number of state)
            posb_run_size = 0
            for run in self.ltl_planner.posb_runs:
                posb_run_size = posb_run_size + len(run)

            # If empty, send warning
            if posb_run_size == 0:
                rospy.logwarn("IRL plugin: Empty set of possible runs")
            # If maximum buffer size is reached, trigger learning and replanning
            elif posb_run_size > self.max_run_buffer_size:
                # Display warning message
                rospy.logwarn("IRL plugin: Possible runs buffer reached maximum size of %i states, triggering learnnig and replanning... "
                               % (self.max_run_buffer_size))
                # Learn and replan
                self.irl_jit(self.ltl_planner.posb_runs)
                # Reset possible runs
                self.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])

            # Save trigger value
            self.prev_learning_trigger = True

        #-----------------------------------------------------------------
        # Else if learning trigger just switched to off, learn and replan
        #-----------------------------------------------------------------
        elif self.prev_learning_trigger and not self.learning_trigger:
            # Learn and replan
            self.irl_jit(self.ltl_planner.posb_runs)
            # Reset possible runs
            self.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])
            # Save trigger value
            self.prev_learning_trigger = False

        #-----------------------------------------------------
        # Else if not learning requested, reset possible runs
        #-----------------------------------------------------
        else:
            # Reset possible runs using current possible states (as no replanning were done)
            self.posb_runs = set([(n,) for n in self.ltl_planner.product.possible_states])


        print "======== POSSIBLE RUNS ========"
        print self.posb_runs

    #------------------------------
   	# Update set of possible runs
   	#------------------------------
    def update_posb_runs(self, prev_runs, ts_node):
        new_runs = set()
        for run in prev_runs:
            f_s = run[-1]
            for t_s in self.ltl_planner.product.successors(f_s):
                if t_s[0] == ts_node:
                    new_run = list(run)
                    new_run.append(t_s)
                    new_runs.add(tuple(new_run))        
        return new_runs

    #------------------------------------------------------------
    # Functions related to Inverse Reinforcement Learning (IRL)
    #-------------------------------------- ---------------------

    # Selects the run which violates the soft task the least
    # Out of the set of all possible product automaton runs, return the one where soft constraint is least violated
    def select_least_violating_run(self, posb_runs):
        least_violating_run = min(posb_runs, key=lambda p: self.compute_path_ac_d(p))
        return least_violating_run

    # Computes the cost of a path(run) in the product automaton
    def compute_path_cost(self, path):
        ac_c = 0
        ac_d = 0
        for i in range(len(path)-1):
            print self.ltl_planner.product[path[i]][path[i+1]]['soft_task_dist']
            ac_d += self.ltl_planner.product[path[i]][path[i+1]]['soft_task_dist']
            ac_c += self.ltl_planner.product[path[i]][path[i+1]]['transition_cost']
        return [ac_c, ac_d]

    # Sets given value of beta parameter (importance of soft task)
    def set_beta(self, beta):
        self.beta = beta
        self.ltl_planner.product.graph['beta'] = beta

    # marginal path (TODO check exactly how this works)
    def margin_opt_path(self, opt_path, beta):
        self.set_beta(beta)
        self.ltl_planner.product.build_full_margin(opt_path)
        #marg_path = dijkstra_path_networkX(self.ltl_planner.product, opt_path[0], opt_path[-1])
        self.run, plantime = dijkstra_plan_networkX(self.ltl_planner.product, self.ltl_planner.gamma)
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
        # Replan
        self.ltl_planner.optimal(style='ready')
        opt_suffix = list(self.run.suffix)
        print 'opt_suffix updated to %s' %str(opt_suffix)
        print '-----------------'
        return beta_seq, match_score

    def compute_path_ac_d(self, path):
        ac_d = 0
        for i in range(len(path)-1):
            ac_d += self.ltl_planner.product[path[i]][path[i+1]]['soft_task_dist']
        return ac_d