#!/usr/bin/env python

import rospy
import re
import random
import time

from armor_api.armor_client import ArmorClient
from armor_api.armor_utils_client import ArmorUtilsClient
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_manipulation_client import ArmorManipulationClient
from threading import Lock

# Import constant name defined to structure the architecture.
from surveillance_robot import architecture_name_mapper as anm

# Import the custom message Statement
from surveillance_robot.msg import Statement

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
		self.map_completed = 0   # set to 1 when the ontology is complete
		self.battery_low = 0     # set to 1 if the battery of the robot is low
		self._locations = []      # list of location objects
		self._doors = []          # list of door objects
		self._corridors = []      # list of corridor objects
		self.prev_loc = 'E'      # previous location, the robot starts from 'E'
		self.goal_loc = ''       # goal location of a move action

		# Initialize useful Armor classes.
		self.client = ArmorClient('mapSurveillance', 'ontoRef')
		self.utils = ArmorUtilsClient(self.client)
		self.query = ArmorQueryClient(self.client)
		self.manipulation = ArmorManipulationClient(self.client)

		# Load the ontology
		self.utils.load_ref_from_file(FILE_PATH, IRI, True, 'PELLET', False)

		# Subscribe to the node that provides statements to build the ontology
		self.subscriber = rospy.Subscriber(anm.TOPIC_STATEMENT, Statement, self.Buildmap_cb)

		# State the robot initial position
		self.manipulation.add_objectprop_to_ind('isIn', 'Robot1', self.prev_loc)

	def Buildmap_cb(self, stat):
		# Callback to the topic used for building the ontology.
		# The message received is of kind 'Statement'
		# If the message received has an empty location, it means that the map is complete
		if stat.location == '':
			self.map_completed = 1
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

	def initialize_map(self):
		# Initialize the ontology. Call this function when all statements have been added to the ontology

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

		# self.utils.save_ref_with_inferences( anm.PATH_TO_PKG+'/topological_map/builded_map.owl' )

	def reason(self):
		# Reason upon the ontology

		self.utils.apply_buffered_changes()
		self.utils.sync_buffered_reasoner()
		# self.client.call('REASON', '', '', [''])
		
	def robot_can_reach(self):
		# Query the ontology and find out which locations can be reached by the robot.
		can_reach = self.query.objectprop_b2_ind('canReach','Robot1')
		can_reach = format(can_reach, '#', '>')

		log_msg = f'The robot is in location {self.prev_loc} and can reach {can_reach}.'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		return can_reach

	def choose_next_location(self, reachable_loc):
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

	def update_robot_position(self):
		# Update the robot position, i.e., change the robot location and accordingly the time stamps of the 
		# robot and the room

		# Replace the robot property isIn, from the previous location to the goal one
		self.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', self.goal_loc, self.prev_loc)
		self.prev_loc = self.goal_loc   # Update the previous location

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