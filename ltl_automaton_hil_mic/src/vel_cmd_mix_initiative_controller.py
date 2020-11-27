#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

#Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemState
from ltl_automaton_msgs.srv import ClosestState, TrapCheck

#==============================================
#         Mix Initative Controller object
#            for Velocity Commands
#                     ---
# Takes as input velocity commands (Twist msg)
# and a distance to trap metric to mix planner
#             and human inputs
#==============================================
class VelCmdMixer(object):
    def __init__(self):
        # Get parameters
        self.load_params()

    #--------------------------------------------------
    # Load controller parameters from parameter server
    #--------------------------------------------------
    def load_params(self):
        # Get epsilon gain from ROS parameters
        self.epsilon = rospy.get_param("gains/epsilon", 1.5)

        # Get deadband from ROS parameters
        self.deadband = rospy.get_param("gain/deadband", 0.2)

        # Get velocity component saturations
        self.max_linear_x_vel = rospy.get_param("gain/max_linear_x_vel", 0.7)
        self.max_linear_y_vel = rospy.get_param("gain/max_linear_y_vel", 0.7)
        self.max_linear_z_vel = rospy.get_param("gain/max_linear_z_vel", 0.7)
        self.max_angular_x_vel = rospy.get_param("gain/max_angular_x_vel", 0.5)
        self.max_angular_y_vel = rospy.get_param("gain/max_angular_y_vel", 0.5)
        self.max_angular_z_vel = rospy.get_param("gain/max_angular_z_vel", 0.5)

        # Node frequency
        self.frequency = rospy.get_param("node_frequency", 50)

        # Get monitored TS state model
        self.state_dimension_name = rospy.get_param("state_dimension_name", "2d_pose_region")

        supported_state_types = ["2d_pose_region",
                                 "3d_pose_region",
                                 "2d_point_region",
                                 "3d_point_region"]
        
        if self.state_dimension_name not in supported_state_types:
            raise ValueError("TS state dimension %s is not supported by velocity command mix initiave controller")

    #---------------------------------------------------
    # Setup subscribers, publishers and service clients
    #---------------------------------------------------
    def set_pub_sub(self):
        # Set closest region service client
        closest_reg_srv = rospy.ServiceProxy("closest_region", ClosestState)

        # Set trap check service client
        trap_cheq_srv = rospy.ServiceProxy("check_for_trap", TrapCheck)

        # Set mix initiave controller output
        mix_vel_cmd_pub = rospy.Publisher("mix_cmd_vel", Twist, queue=50)

        # Set agent TS state subscriber
        rospy.Subscriber("ts_state", TransitionSystemState, self.ts_state_callback ,queue=50)

        # Set planner input subscriber
        rospy.Subscriber("planner_cmd_vel", Twist, self.planner_cmd_callback ,queue=50)

        # Set human input planner
        rospy.Subscriber("teleop_cmd_vel", Twist, self.teleop_cmd_callback, queue=50)

    #-------------------------
    # Agent TS state callback
    #-------------------------
    def ts_state_callback(self, msg):
        # If not using same state model type, print warning and ignore message
        if not self.state_dimension_name in msg.state_dimension_names:
            rospy.logwarn("Received TS state does not include state model type used by velocity command HIL MIC (%s), TS state is of type %s"
                          % (self.state_dimension_name, msg.state_dimension_names))
        # If lenght of states is different from length of state dimension names, message is malformed
        # print warning and ignore message
        if not (len(msg.state_dimension_names) == len(msg.states)):
            rospy.logwarn("Received TS state but number of states: %i doesn't correpond to number of state dimensions: %i",
                          % (len(msg.states),len(msg.state_dimension_names)))
        # Else message is valid, save it
        else:
            self.curr_ts_state = msg

    #--------------------------------
    # Planner velocity command input
    #--------------------------------
    def planner_cmd_callback(self, msg):
        self.planner_input_vel = msg


        #TODO Run
        #IF human input received, run if TS state received as well

    #--------------------------------
    # Human velocity command input
    #--------------------------------
    def teleop_cmd_callback(self, msg):
        self.teleop_input_vel = msg

    #------------------------------------------------
    # Check if risk of trap and get distance to trap
    #------------------------------------------------
    def check_for_trap(self):
        # Get closest region
        closest_reg_req = ClosestStateRequest()
        closest_reg = closest_reg_srv()
        # If service returns a closest state
        if closest_reg.closest_state not "":
            # Create check for trap request from TS state
            ts_state_to_check = self.curr_ts_state
            for i in range(len(self.curr_ts_state.state_dimension_names)):
                # Replace current region by region to check
                if self.curr_ts_state.state_dimension_names[i] == self.state_dimension_name:
                    ts_state_to_check.states[i] = closest_reg.closest_state

            # Populate request and call service to check if closest region would trigger a trap state
            check_for_trap_req = CheckTrapRequest()
            check_for_trap_req.ts_state = ts_state_to_check
            check_for_trap_res = trap_cheq_srv()

            # if IT'S A TRAP! (insert Amiral Ackbar meme)
            if check_for_trap_res.is_connected and check_for_trap_res.is_trap:
                # Return distance to closest region
                return closest_reg.metric

        # If not a trap or cannot check for trap (no closest region or region unconnected)
        return False
           

    def controller(self):
        #print 'telecontrol signal is' + str(self.tele_control)
        tele_control_x = self.tele_control[0]
        tele_control_y = self.tele_control[1]
        tele_magnitude = tele_control_x**2 + tele_control_y**2

        if tele_magnitude >= self.deadband:
            #print '--- Human inputs detected ---'
            if self.dist_to_trap >=0:
                # print 'Distance to trap states in product: %.2f' %self.dist_to_trap
                self.mix_control = self.smooth_mix(self.tele_control, self.navi_control, self.dist_to_trap)
                # print 'mix_control: %s ||| navi_control: %s ||| tele_control: %s ||| gain: %.2f' %(self.mix_control, self.navi_control, self.tele_control, self.gain)
            else:
                #if no obstacle is close, use human command
                # print 'No trap states are close'
                self.dist_to_trap = 1000
                self.mix_control = self.smooth_mix(self.tele_control, self.navi_control, self.dist_to_trap)
        else:
            #print 'No Human inputs. Autonomous controller used.'
            self.mix_control = self.navi_control

    #
    #
    #
    def smooth_mix(self, tele_control, navi_control, dist_to_trap):
        tele_control[0] = self.saturate(tele_control[0], max_linear_x_vel)
        tele_control[1] = self.saturate(tele_control[1], max_linear_y_vel)
        tele_control[2] = self.saturate(tele_control[2], max_linear_z_vel)

        self.gain = rho(self.dist_to_trap-self.ds)/(rho(self.dist_to_trap-self.ds)+rho(self.epsilon +self.ds-self.dist_to_trap))
        #print 'human-in-the-loop gain is ' + str(self.gain)
        mix_control = [0.0,0.0,0.0]
        mix_control[0] = (1-self.gain)*navi_control[0] + self.gain*tele_control[0]
        mix_control[1] = (1-self.gain)*navi_control[1] + self.gain*tele_control[1]
        mix_control[2] = navi_control[2]
        return mix_control

    def saturate(self, command, max_value):
        if command > max_value:
            command = max_value
        else if command < -max_value:
            command = -max_value
        return command

    #-------------------------------------
    # Rho function needed for mix formula
    #-------------------------------------
    def rho(s):
    if (s > 0):
        return np.exp(-1.0/s)
    else:
        return 0


#============================
#            Main            
#============================
if __name__ == '__main__':
    rospy.init_node('vel_cmd_hil_mic',anonymous=False)
    try:
        vel_cmd_hil_mic = VelCmdMixer()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("Velocity Command HIL MIC: %s" %(e))
        sys.exit(0)