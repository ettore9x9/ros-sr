#!/usr/bin/env python

import rospy
import re
import random
import time
import actionlib

from armor_api.armor_client import *
from threading import Lock

# Import constant name defined to structure the architecture.
from surveillance_robot import architecture_name_mapper as anm
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest
from surveillance_robot.msg import *

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_STATE_MACHINE

# Useful information to load the ontology
FILE_PATH = anm.PATH_TO_PKG+'/topological_map/topological_map.owl'
IRI = 'http://bnc/exp-rob-lab/2022-23'

def format(oldlist, start, end):
	# Function to format a list of strings.
	# It cuts the elements of oldlist between the strings start and end, returning the formatted list.
	newlist = []
	for string in oldlist:
		newlist.append(re.search(start+'(.+?)'+end, string).group(1))

	return newlist

# This class is the state machine helper. It stores useful variables, functions and ros callbacks.
class helper():

	def __init__(self):
		# Initialize class variables
		self.map_completed = 0                  # set to 1 when the ontology is complete
		self.battery_low = False                # set to True if the battery of the robot is low
		self._locations = []                    # list of location objects
		self._doors = []                        # list of door objects
		self._corridors = []                    # list of corridor objects
		self.waypoints = []
		self.prev_loc = anm.STARTING_LOCATION   # previous location, initialized with the starting one
		self.goal_loc = ''                      # goal location of a move action
		self.mutex_map = Lock()
		self.mutex_battery = Lock()
		self.actual_point = Point()
		self.actual_point.x = 0
		self.actual_point.y = 0

		# Initialize useful Armor classes.
		self.client = ArmorClient('mapSurveillance', 'ontoRef')
		self.utils = ArmorUtilsClient(self.client)
		self.query = ArmorQueryClient(self.client)
		self.manipulation = ArmorManipulationClient(self.client)

		# Load the ontology
		self.utils.load_ref_from_file(FILE_PATH, IRI, True, 'PELLET', False)

		# Subscribe to the topic that provides statements to build the ontology
		self.statement_sub = rospy.Subscriber(anm.TOPIC_STATEMENT, Statement, self._Buildmap_cb)

		# Subscribe to the topic that controls the battery level
		self.battery_sub = rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self._Battery_cb)

		rospy.wait_for_service(anm.SERVICE_RECHARGING)
		self.client_recharge = rospy.ServiceProxy(anm.SERVICE_RECHARGING, SetBool)

		self.client_planner = actionlib.SimpleActionClient(anm.ACTION_PLANNER, PlanAction)
		self.client_controller = actionlib.SimpleActionClient(anm.ACTION_CONTROLLER, ControlAction)

		# State the robot initial position
		self.manipulation.add_objectprop_to_ind('isIn', 'Robot1', self.prev_loc)

	def _Buildmap_cb(self, stat):
		# Callback to the topic used for building the ontology.
		# The message received is of kind 'Statement'
		# If the message received has an empty location, it means that the map is complete
		if stat.location == '':
			self.mutex_map.acquire()
			self.map_completed = 1
			self.mutex_map.release()
		else:
			# Add the statement to the ontology.
			self.manipulation.add_objectprop_to_ind('hasDoor', stat.location, stat.door)

			# Update the lists of locations and doors
			if stat.location not in self._locations:
				self._locations.append(stat.location)

			if stat.door not in self._doors:
				self._doors.append(stat.door)

			log_msg = f'Statement `Door {stat.door} is in location {stat.location}`added to the ontology.'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	def _Battery_cb(self, msg):
		self.mutex_battery.acquire()
		self.battery_low = msg.data
		self.mutex_battery.release()
		if msg.data == True:
			log_msg = f'Battery of the robot low, need recharging'
		if msg.data == False:
			log_msg = f'Battery of the robot completely charged'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	def build_the_map(self):
		r = rospy.Rate(10)  # 10hz
		while not rospy.is_shutdown():
		    self.mutex_map.acquire()
		    try:
		        if self.map_completed == True:
		            # Initialize the ontology when all statements have been added to the ontology

		            # Disjoint all individuals
		            self.client.call('DISJOINT', 'IND', '', self._locations+self._doors)
		            now = str(int(time.time()))   # Current time
		            # Start the timer in all locations, when it expires, the location is called URGENT by the reasoner.
		            for loc in self._locations:
		            	self.manipulation.add_dataprop_to_ind( 'visitedAt', loc, 'Long', now )
		            self.reason()   # Call the reasoner
		            # Query the ontology for all corridors
		            self._corridors = self.query.ind_b2_class('CORRIDOR')
		            self._corridors = format(self._corridors, '#', '>')
		            log_msg = f'Map initialized, the corridors are: {self._corridors}'
		            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		            return
		    finally:
		        self.mutex_map.release()
		    r.sleep()

	def query_the_ontology(self):
		self.reason()

		can_reach = self.query.objectprop_b2_ind('canReach','Robot1')
		can_reach = format(can_reach, '#', '>')

		log_msg = f'The robot is in location {self.prev_loc} and can reach {can_reach}.'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		self.mutex_battery.acquire()
		try:
		    if self.battery_low == True and anm.RECHARGING_LOCATION in can_reach:
		        self.goal_loc = anm.RECHARGING_LOCATION
		        return 'battery_low'
		finally:
		    self.mutex_battery.release()

		self.goal_loc = self.surveillance_policy(can_reach)
		return 'reach_location'

	def recharge(self):
		req = SetBoolRequest()
		req.data = True
		resp = self.client_recharge(req)
		
		self.mutex_battery.acquire()
		self.battery_low = False
		self.mutex_battery.release()

	def plan_path(self):
		next_point = Point()
		next_point.x = random.uniform(0,10)
		next_point.y = random.uniform(0,10)

		request = PlanActionGoal()
		request.goal.target = next_point
		request.goal.actual = self.actual_point

		self.client_planner.wait_for_server()
		self.client_planner.send_goal(request.goal)
		self.client_planner.wait_for_result()

		self.waypoints = (self.client_planner.get_result()).via_points

	def control_robot(self):
		request = ControlActionGoal()
		request.goal.via_points = self.waypoints
		self.client_controller.send_goal(request.goal)
		self.client_controller.wait_for_result()
		self.actual_point = (self.client_controller.get_result()).reached_point

		self.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', self.goal_loc, self.prev_loc)
		self.prev_loc = self.goal_loc   # Update the location

		now = str(int(time.time()))   # Current time

		# Last time that the robot moves
		before_robot = self.query.dataprop_b2_ind('now', 'Robot1')
		before_robot = format(before_robot, '"', '"')

		# Update the robot time
		self.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', now, before_robot[0])

		# Update the location time when the robot visits it
		before_loc = self.query.dataprop_b2_ind('visitedAt', self.goal_loc)
		before_loc = format(before_loc, '"', '"')
		self.manipulation.replace_dataprop_b2_ind('visitedAt', self.goal_loc, 'Long', now, before_loc[0])

		log_msg = f'The robot has reached location {self.goal_loc} at time {now}'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	def reason(self):
		# Reason upon the ontology
		self.utils.apply_buffered_changes()
		self.utils.sync_buffered_reasoner()

	def surveillance_policy(self, reachable_loc):
		# Choose the next location among the reachable ones.
		# It performs the following surveillance policy:
		# - It mainly stays on corridors
		# - If a reachable location is URGENT, it should visit it.

		# Shuffle the reachable location, not to have a preference between them.
		random.shuffle(reachable_loc)

		# Query for all URGENT locations
		urgent_loc = self.query.ind_b2_class('URGENT')
		urgent_loc = format(urgent_loc, '#', '>')

		log_msg = f'The urgent locations are {urgent_loc}'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		# If there is a reachabe location which is URGENT, return that location
		for loc in reachable_loc:
			if loc in urgent_loc:
				return loc

		# If there is a reachable location which is a CORRIDOR, return that location
		for loc in reachable_loc:
			if loc in self._corridors:
				return loc

		# Return a randomic location (because of shuffling them)
		return reachable_loc[0]