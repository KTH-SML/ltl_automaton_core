#!/usr/bin/env python
import roslib
import numpy
import Queue
import rospy
import sys
import time
import numpy as np
from copy import deepcopy

POSITION_ERROR = 0.45
ORIENTATION_ERROR = 0.3


from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PolygonStamped, Point32, PointStamped, PoseArray, Pose, Point

from std_msgs.msg import Bool, String, Float64

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from rqt_simulation_msgs.msg import Sense, TemporaryTask, RoiArray

from math import pi as PI
from math import atan2, sin, cos, sqrt

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, euler_from_matrix

from ts import MotionFts, ActionModel, MotActModel

class LtlPlannerNode(object):
    def __init__(self):

        rospy.init_node('ltl_planner')

        print('Here')
        
        #Initialize variable
        self.node_name = "LTL Planner"
        self.active = False
        self.hard_task = ''
        self.soft_task = ''

        #Get parameters from parameter server
        self.robot_name = rospy.get_param('robot_name')
        self.agent_type = rospy.get_param('agent_type')
        self.navigation_type = rospy.get_param('navigation_type')
        self.localization_topic = rospy.get_param('localization_topic')
        ap = rospy.get_param('robot_motion/ap')

        #Initialize plan timer
        self.replan_timer = rospy.Time.now()

        #Header for debug print
        self.header = '['+self.robot_name+'] '

        #Initialize the pose
        self.last_current_pose = PoseStamped()

        #Load robot model from config file
        robot_model = FTSLoader(scenario_file)
        [self.robot_motion, self.curr_pose, self.robot_action, self.robot_task] = robot_model.robot_model

        self.robot_motion.set_initial(self.curr_pose)
        rospy.loginfo('%s : %s : The inital hard task is: %s' % (self.node_name, self.robot_name, self.hard_task))
        rospy.loginfo('%s : %s : The inital soft task is: %s' % (self.node_name, self.robot_name, self.soft_task))


    
#==============================
#             Main
#==============================
if __name__ == '__main__':

    ltl_planner_node = LtlPlannerNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

    # rospy.init_node('ltl_planner',anonymous=False)
    # ltl_planner_node = LtlPlannerNode()
    # rospy.spin()
