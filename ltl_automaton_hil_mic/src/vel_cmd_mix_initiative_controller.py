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

        # Setup subscribers and publishers
        self.set_pub_sub()

    #--------------------------------------------------
    # Load controller parameters from parameter server
    #--------------------------------------------------
    def load_params(self):
        self.curr_ts_state = None

        # Get epsilon gain from ROS parameters
        self.epsilon = rospy.get_param("~epsilon", 1.5)

        # Get safety distance from ROS parameters
        self.ds = rospy.get_param("~ds", 1.2)

        # Get deadband from ROS parameters
        self.deadband = rospy.get_param("~deadband", 0.2)

        # Get human input timeout (in seconds) from ROS parameters
        self.timeout = rospy.get_param("~timeout", 0.2)
        # Init last received human input time by deducting timeout from current time
        # to ensure timeout at initialization
        self.last_received_human_input = rospy.Time.now() - rospy.Duration.from_sec(self.timeout)

        # Get velocity component saturations
        self.max_linear_x_vel = rospy.get_param("~max_linear_x_vel", 0.7)
        self.max_linear_y_vel = rospy.get_param("~max_linear_y_vel", 0.7)
        self.max_linear_z_vel = rospy.get_param("~max_linear_z_vel", 0.7)
        self.max_angular_x_vel = rospy.get_param("~max_angular_x_vel", 0.5)
        self.max_angular_y_vel = rospy.get_param("~max_angular_y_vel", 0.5)
        self.max_angular_z_vel = rospy.get_param("~max_angular_z_vel", 0.5)

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
        mix_vel_cmd_pub = rospy.Publisher("mix_cmd_vel", Twist, queue_size=50)

        # Set agent TS state subscriber
        rospy.Subscriber("ts_state", TransitionSystemState, self.ts_state_callback ,queue_size=50)

        # Set human input planner
        rospy.Subscriber("teleop_cmd_vel", Twist, self.teleop_cmd_callback, queue_size=50)

        # Set planner input subscriber
        rospy.Subscriber("planner_cmd_vel", Twist, self.planner_cmd_callback ,queue_size=50)

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
            rospy.logwarn("Received TS state but number of states: %i doesn't correpond to number of state dimensions: %i"
                          % (len(msg.states),len(msg.state_dimension_names)))
        # Else message is valid, save it
        else:
            self.curr_ts_state = msg

    #--------------------------------
    # Planner velocity command input
    #--------------------------------
    def planner_cmd_callback(self, msg):
        self.planner_input_vel = msg

        # If human input received recently and TS state is known
        if (rospy.Time.now() - self.last_received_human_input < self.timeout) and (self.curr_ts_state):
            # Run controller mix and publish
            self.mix_vel_cmd_pub.publish(self.control_mixer())

    #--------------------------------
    # Human velocity command input
    #--------------------------------
    def teleop_cmd_callback(self, msg):
        self.teleop_input_vel = msg
        self.last_received_human_input = rospy.Time.now()

    #------------------------------------------------
    # Check if risk of trap and get distance to trap
    #------------------------------------------------
    def check_for_trap(self):
        # Get closest region
        closest_reg_req = ClosestStateRequest()
        closest_reg = closest_reg_srv()
        # If service returns a closest state
        if closest_reg.closest_state:
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
           

    def control_mixer(self, teleop_vel_cmd, planner_vel_cmd):
        #print 'telecontrol signal is' + str(self.tele_control
        tele_magnitude = math.sqrt(teleop_vel_cmd.linear.x**2
                                 + teleop_vel_cmd.linear.y**2
                                 + teleop_vel_cmd.linear.z**2)

        if tele_magnitude >= self.deadband:
            #print '--- Human inputs detected ---'

            # Check for trap
            dist_to_trap = self.check_for_trap()

            # If dist to trap returns a value, compute mix
            if dist_to_trap:
                # print 'Distance to trap states in product: %.2f' %self.dist_to_trap
                teleop_vel_cmd = self.bound_vel_cmd(teleop_vel_cmd)
                self.mix_control = self.smooth_mix(teleop_vel_cmd, planner_vel_cmd, dist_to_trap, self.ds, self.epsilon)
                # print 'mix_control: %s ||| navi_control: %s ||| tele_control: %s ||| gain: %.2f' %(self.mix_control, self.navi_control, self.tele_control, self.gain)
            
            # If distance to trap returns false, no trap is close, use human input
            else:
                #if no obstacle is close, use human command
                # print 'No trap states are close'
                teleop_vel_cmd = self.bound_vel_cmd(teleop_vel_cmd)
                self.mix_control = self.teleop_vel_cmd
        else:
            #print 'No Human inputs. Autonomous controller used.'
            self.mix_control = self.navi_control

        return self.mix_control

    #------------------------------------------------------------------------
    # Return the mixed velocity command given the distance to trap and gains
    #------------------------------------------------------------------------
    def smooth_mix(self, tele_control, navi_control, dist_to_trap, ds, epsilon):
        # Compute gain using epsilon, dist to trap and ds
        gain = rho(dist_to_trap-ds)/(rho(dist_to_trap-ds)+rho(epsilon +self.ds-dist_to_trap))
        #print 'human-in-the-loop gain is ' + str(self.gain)

        # Mix velocity commands by using previously calculated gain
        mix_vel_cmd = Twist()
        mix_vel_cmd.linear.x = (1 - gain) * navi_control.linear.x + gain * tele_control.linear.x
        mix_vel_cmd.linear.y = (1 - gain) * navi_control.linear.y + gain * tele_control.linear.y
        mix_vel_cmd.linear.z = (1 - gain) * navi_control.linear.z + gain * tele_control.linear.z
        mix_vel_cmd.angular.x = (1 - gain) * navi_control.angular.x + gain * tele_control.angular.x
        mix_vel_cmd.angular.y = (1 - gain) * navi_control.angular.y + gain * tele_control.angular.y
        mix_vel_cmd.angular.z = (1 - gain) * navi_control.angular.z + gain * tele_control.angular.z

        return mix_vel_cmd

    #-----------------------------------------------------------
    # Bound the minimum and maximum value of a velocity command
    #-----------------------------------------------------------
    def bound_vel_cmd(self, vel_twist_msg):
        vel_twist_msg.linear.x = self.bound(vel_twist_msg.linear.x, max_linear_x_vel)
        vel_twist_msg.linear.y = self.bound(vel_twist_msg.linear.y, max_linear_y_vel)
        vel_twist_msg.linear.z = self.bound(vel_twist_msg.linear.z, max_linear_z_vel)
        vel_twist_msg.angular.x = self.bound(vel_twist_msg.angular.x, max_angular_x_vel)
        vel_twist_msg.angular.y = self.bound(vel_twist_msg.angular.y, max_angular_y_vel)
        vel_twist_msg.angular.z = self.bound(vel_twist_msg.angular.z, max_angular_z_vel)

        return vel_twist_msg

    #---------------
    # Bound a value
    #---------------
    def bound(self, command, max_value):
        if command > max_value:
            command = max_value
        elif command < -max_value:
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