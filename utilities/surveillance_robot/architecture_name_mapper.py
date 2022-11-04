#!/usr/bin/env python
"""
.. module:: architecture_name_mapper
  :platform: Unix 
  :synopsis: Python module for the architecture name mapper
.. moduleauthor:: Ettore Sani 5322242@studenti.unige.it

This module stores the names of all nodes, topics and services of the architecture.

"""

### IMPORTS ###
import rospy
import rospkg

### GLOBAL ###
# The name of a boolean parameter to active random testing.
# If the value is `False` stimulus are generated sequentially and equally spaced in time.
# Instead, stimulus will be generate in a random order and with a random delay if `True`.
# In the latter case, the architecture also requires all the parameters 
# with a the scope `test/random_sense/*`, which are not used if `False`.
PARAM_RANDOM_ACTIVE = 'test/random_sense/active'

# Name of the location where the robot starts.
STARTING_LOCATION = 'E'

# Name of the location where the robot can recharge, it could be different from the starting one.
RECHARGING_LOCATION = 'E'

# Initial position of the robot in the 2D space.
INITIAL_2DPOINT = [0.0,0.0]

# Characteristic dimension of the environment.
DIMENSION = 10
# ---------------------------------------------------------

# The name of the state machine node
NODE_STATE_MACHINE = 'state_machine'
# ---------------------------------------------------------

# The name of the node representing the shared knowledge required for this scenario.
NODE_BATTERY_MANAGER = 'battery_manager'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'

# The name of the service to recharge the robot's battery.
SERVICE_RECHARGING = 'state/recharging'

# Time needed for recharging completely the robot.
PARAM_RECHARGING_TIME = 'test/recharging_time'

# The duration of the robot's battery.
# It should be a list `[min_time, max_time]`, and the low battery level
# will occur after a random number of seconds within such an interval.
PARAM_BATTERY_TIME = 'test/random_sense/battery_time'
# ---------------------------------------------------------

# The name of the planner node.
NODE_PLANNER = 'planner'

# The name of the action server solving the motion planning problem.
ACTION_PLANNER = 'motion/planner'

# The number of points in the plan. It should be a list `[min_n, max_n]`,
# Where the number of points is a random value in the interval [`min_n`, `max_n`).
PARAM_PLANNER_POINTS = 'test/random_plan_points'

# The delay between the computation of the next via points.
# It should be a list `[min_time, max_time]`, and the computation will 
# last for a random number of seconds in such an interval.
PARAM_PLANNER_TIME = 'test/random_plan_time'
# -------------------------------------------------

# The name of the controller node.
NODE_CONTROLLER = 'controller'

# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'

# The time required to reach a via points.
# It should be a list `[min_time, max_time]`, and the time to reach a
# via point will be a random number of seconds in such an interval.
PARAM_CONTROLLER_TIME = 'test/random_motion_time'
# -------------------------------------------------

# The name of the node that publishes the statements about the environment.
NODE_STATEMENT_PUB = 'statement_pub'

# The name of the topic where the statements are published.
TOPIC_STATEMENT = 'map/statement'

# The name of a boolean parameter to active random generation of statements.
# If the value is `False` the statements are generated in sequence. 
# Instead, random statement sequence will be generate if `True`.
PARAM_RANDOM_STATEMENT_ACTIVE = 'test/random_sense/active'

# The delay between statements.
# It should be a list `[min_time, max_time]`, and the new statement
# will occur after a random number of seconds within such an interval.
PARAM_STATEMENT_TIME = 'test/random_sense/statement_time'
# -------------------------------------------------

# Get the file path for surveillance_robot.
rospack = rospkg.RosPack()

# The absolute path to the surveillance_robot package.
PATH_TO_PKG = rospack.get_path('surveillance_robot')

# Useful information to load the ontology
FILE_PATH = PATH_TO_PKG+'/topological_map/topological_map.owl'
IRI = 'http://bnc/exp-rob-lab/2022-23'

# -------------------------------------------------

### FUNCTIONS ###
def tag_log(msg, producer_tag):
    """Function used to label each log with a producer tag.

    Args:
        msg (str): message to print on screen
        producer_tag (str): tag name of the producer of the message

    Returns:
        string (str): formatted string

    """
    return '@%s>> %s' % (producer_tag, msg)
