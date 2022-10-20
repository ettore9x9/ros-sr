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

class helper():
	def __init__(self):
		self.client = ArmorClient('mapSurveillance', 'ontoRef')
		self.utils = ArmorUtilsClient(self.client)
		self.query = ArmorQueryClient(self.client)
		self.manipulation = ArmorManipulationClient(self.client)

		owl_file_path = anm.PATH_TO_PKG+'/topological_map/topological_map.owl'
		iri = 'http://bnc/exp-rob-lab/2022-23'
		self.utils.load_ref_from_file(owl_file_path, iri, True, 'PELLET', False)

		self.map_completed = 0
		self.battery_low = 0

		self.subscriber = rospy.Subscriber(anm.TOPIC_STATEMENT, Statement, self.Buildmap_cb)

		self.locations = []
		self.doors = []
		self.corridors = []
		self.prev_loc = 'E'
		self.goal_loc = ''
		self.manipulation.add_objectprop_to_ind('isIn', 'Robot1', self.prev_loc)

	def Buildmap_cb(self, stat):
	    if stat.location == '':
	        self.map_completed = 1
	    else:
	        self.client.call('ADD', 'OBJECTPROP', 'IND', ['hasDoor', stat.location, stat.door])

	        if stat.location not in self.locations:
	        	self.locations.append(stat.location)

	        if stat.door not in self.doors:
	        	self.doors.append(stat.door)

	        log_msg = f'Statement `Door {stat.door} is in location {stat.location}`added to the ontology.'
	        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	def initialize_map(self):

		self.client.call('DISJOINT', 'IND', '', self.locations+self.doors)
		now = str(int(time.time()))
		for loc in self.locations:
			self.manipulation.add_dataprop_to_ind( 'visitedAt', loc, 'Long', now )

		self.reason()
		self.corridors = self.query.ind_b2_class('CORRIDOR')

	def reason(self):
		# self.utils.apply_buffered_changes()
		# self.utils.sync_buffered_reasoner()
		self.client.call('REASON', '', '', [''])
		self.utils.save_ref_with_inferences( anm.PATH_TO_PKG+'/topological_map/builded_map.owl' )
		
	def robot_can_reach(self):
		location_to_format = self.query.objectprop_b2_ind('canReach','Robot1')
		location = []

		for string in location_to_format:
		    location.append(re.search('#(.+?)>', string).group(1))
		
		log_msg = f'The robot can reach locations {location}.'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		return location

	def choose_next_location(self, reachable_loc):

		random.shuffle(reachable_loc)
		urgent_loc = []
		urgent_loc_to_format = self.query.ind_b2_class('URGENT')

		for string in urgent_loc_to_format:
		    urgent_loc.append(re.search('#(.+?)>', string).group(1))

		log_msg = f'The robot is in location {self.prev_loc} and can reach {reachable_loc}; the urgent locations are {urgent_loc}'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		for loc in reachable_loc:
			if loc in urgent_loc:
				return loc

		for loc in reachable_loc:
			if loc in self.corridors:
				return loc

		return reachable_loc[0]

	def update_robot_position(self):

		self.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', self.goal_loc, self.prev_loc)
		self.prev_loc = self.goal_loc

		now = str(int(time.time()))
		before_robot = self.query.dataprop_b2_ind('now', 'Robot1')
		before_robot = re.search('"(.+?)"', before_robot[0]).group(1)
		self.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', now, before_robot)

		before_loc = self.query.dataprop_b2_ind('visitedAt', self.goal_loc)

		if before_loc == []:
			self.manipulation.add_dataprop_to_ind('visitedAt', self.goal_loc, 'Long', now)

		else:
			before_loc = re.search('"(.+?)"', before_loc[0]).group(1)
			self.manipulation.replace_dataprop_b2_ind('visitedAt', self.goal_loc, 'Long', now, before_loc)

		log_msg = f'The robot has reached location {self.goal_loc} at time {now}'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))