#!/usr/bin/env python
"""
.. module:: state_machine_helper
  :platform: Unix 
  :synopsis: Python module for the helper class, used by the module :mod:`state_machine`
.. moduleauthor:: Ettore Sani 5322242@studenti.unige.it

This module manages subscribers and client requests for the module :mod:`state_machine` through the helper class.
It provides several useful methods to exchange information with the ontology, thanks to the api provided by the `*armor_api* <https://github.com/EmaroLab/armor_py_api>`_ library.
This module has several interfaces with orther modules, such as :mod:`battery_manager`, :mod:`find_qr`, :mod:`planner` and :mod:`controller` modules.

Subscribes to:
  /map/statement
  /state/battery_low

Client of:
  /state/recharging

Action client of:
  /motion/planner
  /motion/controller

"""

### IMPORTS ###
import rospy
import re
import random
import time
import actionlib

from armor_api.armor_client import *
from threading import Lock
from surveillance_robot import architecture_name_mapper as anm

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest
from surveillance_robot.msg import *

### GLOBAL ###
LOG_TAG = anm.NODE_STATE_MACHINE   # Tag for identifying logs producer.

### FUNCTIONS ###
def format(oldlist, start, end):
	"""Function to format a list of strings.
	It cuts the elements of the old list between the start and end characters, returning the formatted list.
	It relies on the `re <https://docs.python.org/3/library/re.html>`_ python library.

    Args:
       oldlist (str[]): The old list of strings to format.
       start (char): Starting character to start cutting in every string.
       end (char): Ending character to end cutting in every string.

    Returns:
       newlist (str[]): The formatted list of strings.

    """
	newlist = []
	for string in oldlist:
		newlist.append(re.search(start+'(.+?)'+end, string).group(1))
	return newlist

### CLASSES ###
class helper():
	"""This class is the state machine helper. It stores useful variables, functions and ros callbacks.
	It is widely used by the :mod:`scripts.state_machine` module.

	"""
	def __init__(self):
		# Initialize variables.
		self.map_completed = False              # set to True when the ontology is complete
		self.battery_low = False                # set to True if the battery of the robot is low
		self._locations = []                    # list of location objects
		self._doors = []                        # list of door objects
		self._corridors = []                    # list of corridor objects
		self.waypoints = []                     # list of waypoints to reach the next location
		self.goal_loc = ''                      # goal location of a move action
		self.mutex_map = Lock()                 # mutex for the self.map_completed variable
		self.mutex_battery = Lock()             # mutex for the self.battery low variable
		self.actual_point = Point()             # 2D point of the robot in the environment

		# Get the starting 2D position and location.
		self.actual_point.x = anm.INITIAL_2DPOINT[0]   # starting x coordinate
		self.actual_point.y = anm.INITIAL_2DPOINT[1]   # starting y coordinate
		self.prev_loc = anm.STARTING_LOCATION          # starting location

		# Initialize useful Armor classes.
		self.client = ArmorClient('mapSurveillance', 'ontoRef')
		self.utils = ArmorUtilsClient(self.client)
		self.query = ArmorQueryClient(self.client)
		self.manipulation = ArmorManipulationClient(self.client)

		# Load the ontology.
		self.utils.load_ref_from_file(anm.FILE_PATH, anm.IRI, True, 'PELLET', False)

		# Subscribe to the topic that provides statements to build the ontology.
		self.statement_sub = rospy.Subscriber(anm.TOPIC_STATEMENT, Statement, self._Buildmap_cb)

		# Subscribe to the topic that controls the battery level.
		self.battery_sub = rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self._Battery_cb)

		# Create a client for the recharging service.
		rospy.wait_for_service(anm.SERVICE_RECHARGING)
		self.client_recharge = rospy.ServiceProxy(anm.SERVICE_RECHARGING, SetBool)

		# Create an action client for the planner action service.
		self.client_planner = actionlib.SimpleActionClient(anm.ACTION_PLANNER, PlanAction)

		# Create an action client for the controller action service.
		self.client_controller = actionlib.SimpleActionClient(anm.ACTION_CONTROLLER, ControlAction)
		self.client_controller.wait_for_server()

		# State the robot initial position
		self.manipulation.add_objectprop_to_ind('isIn', 'Robot1', self.prev_loc)
		self.client_planner.wait_for_server()

	def _Buildmap_cb(self, stat):
		"""Callback of the topic */map/statement* used for building the ontology.
		The message received is of kind 'Statement' and contains a location coupled with a door.
		If the message is empty, it means that the map is completed, so it raises the map_completed flag.

		"""
		# If the message has an empty location.
		if stat.location == '':
			self.mutex_map.acquire()    # take the map mutex
			self.map_completed = True   # the map is completed
			self.mutex_map.release()    # release the map mutex
		else:
			# Add the statement to the ontology.
			self.manipulation.add_objectprop_to_ind('hasDoor', stat.location, stat.door)

			# Update the lists of locations and doors.
			if stat.location not in self._locations:
				self._locations.append(stat.location)
			if stat.door not in self._doors:
				self._doors.append(stat.door)

			log_msg = f'Statement `Door {stat.door} is in location {stat.location}`added to the ontology.'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	def _Battery_cb(self, msg):
		"""Callback of the topic */state/battery_low*, that triggers when the robot needs to recharge
		or when the robot is completely charged. It changes the battery_low flag accordingly with the
		message received.

		"""
		self.mutex_battery.acquire()   # take the battery mutex
		self.battery_low = msg.data    # change the flag of battery low with the received message
		self.mutex_battery.release()   # release the battery mutex
		if msg.data == True:
			log_msg = f'Battery of the robot low, need recharging'
		if msg.data == False:
			log_msg = f'Battery of the robot completely charged'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	def build_the_map(self):
		"""Function used by the :class:`scripts.state_machine.Buildmap` class; it waits until the 
		map is completed, than disjoints all individuals in the ontology, asks the reasoner to reason with
		the given ontology and queries it for locations that are corridors.

		"""
		r = rospy.Rate(10)  # run this code with a frequency of 10Hz
		while not rospy.is_shutdown():
		    self.mutex_map.acquire()   # take the map mutex
		    try:
		        if self.map_completed == True:   # if the map is completed

		            # Disjoint all individuals.
		            self.client.call('DISJOINT', 'IND', '', self._locations+self._doors)
		            now = str(int(time.time()))   # current time

		            # Start the timer in all locations, when it expires, the location is called URGENT by the reasoner.
		            for loc in self._locations:
		            	self.manipulation.add_dataprop_to_ind( 'visitedAt', loc, 'Long', now )

		            self.reason()   # call the reasoner

		            # Query the ontology for all corridors.
		            self._corridors = self.query.ind_b2_class('CORRIDOR')
		            self._corridors = format(self._corridors, '#', '>')

		            log_msg = f'Map initialized, the corridors are: {self._corridors}'
		            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		            return
		    finally:
		        self.mutex_map.release()   # release the map mutex

		    r.sleep()   # sleep until the next cycle

	def query_the_ontology(self):
		"""Function used by the :class:`scripts.state_machine.Query` class; it asks the ontology about the
		locations that the robot can reach and chooses one of them according to the battery level of the robot 
		and on the surveillance policy.

		Returns:
			'battery_low' (str): string to have a battery_low transition
			'reach_location' (str): string to have a reach_location transition

		"""
		self.reason()   # ask the reasoner to reason about the ontology

		# Query the ontology about the locations that the robot can reach.
		can_reach = self.query.objectprop_b2_ind('canReach','Robot1')
		can_reach = format(can_reach, '#', '>')

		log_msg = f'The robot is in location {self.prev_loc} and can reach {can_reach}.'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		self.mutex_battery.acquire()   # take the battery mutex
		try:
			# If the robot has low battery, and it can reach the recharging location.
		    if self.battery_low == True and anm.RECHARGING_LOCATION in can_reach:
		        self.goal_loc = anm.RECHARGING_LOCATION   # the goal location is the recharging one
		        return 'battery_low'
		finally:
		    self.mutex_battery.release()   # release the battery mutex

		# Call the surveillance policy for computing the next location.
		self.goal_loc = self.surveillance_policy(can_reach)

		if self.goal_loc == False:        # if the robot can not reach any other location
			self.goal_loc = self.prev_loc   # remain in the same location

		return 'reach_location'

	def recharge(self):
		"""Function used by the :class:`scripts.state_machine.WaitFull` class; it makes a request to the 
		service that recharges the battery. This service is blocking, and returns only when the battery is fully charged.
		Then, it puts down the battery_low flag.

		"""
		req = SetBoolRequest()             # initialize the service request
		req.data = True                    # initialize the data field of the request
		resp = self.client_recharge(req)   # send the request to the service
		
		self.mutex_battery.acquire()       # take the battery mutex
		self.battery_low = False           # set the battery_low flag to false
		self.mutex_battery.release()       # release the battery mutex

	def plan_path(self):
		"""Function used by the :class:`scripts.state_machine.Planner` class; it makes a request to the action
		service that plan the viapoints between the actual and the goal positions. The goal position is choosen
		randomically. It waits until the path is planned, then returns.

		"""

		request = PlanActionGoal()                       # initialize the action service request
		request.goal.target = self.goal_loc              # fill the goal point of the action service request

		self.client_planner.send_goal(request.goal)      # send the request to the action server
		self.client_planner.wait_for_result()            # wait for the result of the action service

		# Store the planned viapoints in a class variable, it will be used by the controller.
		self.waypoints = (self.client_planner.get_result()).via_points

	def control_robot(self):
		"""Function used by the :class:`scripts.state_machine.Controller` class; it makes a request to the action
		service that controls the robot through viapoints. It waits until the goal point has been reached, updates
		the ontology and then returns.

		"""
		request = ControlActionGoal()                    # initialize the action service request
		request.goal.via_points = self.waypoints         # fill the action service request with the stored viapoints
		self.client_controller.send_goal(request.goal)   # send the request to the action service
		self.client_controller.wait_for_result()         # wait until the action service finished
		self.actual_point = (self.client_controller.get_result()).reached_point   # update the actual position of the robot

		# Update the ontology.
		self.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', self.goal_loc, self.prev_loc)   # update the robot's location in the ontology		
		self.prev_loc = self.goal_loc                                # update the current robot location
		now = str(int(time.time()))                                  # current time
		before_robot = self.query.dataprop_b2_ind('now', 'Robot1')   # query the ontology for the last time that the robot moves
		before_robot = format(before_robot, '"', '"')                # format the time
		self.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', now, before_robot[0])   # update the robot time
		before_loc = self.query.dataprop_b2_ind('visitedAt', self.goal_loc)   # update the location time when the robot visits it
		before_loc = format(before_loc, '"', '"')                    # format the time
		self.manipulation.replace_dataprop_b2_ind('visitedAt', self.goal_loc, 'Long', now, before_loc[0])   # update the time when the robot has visited the goal location

		log_msg = f'The robot has reached location {self.goal_loc} at time {now}'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	def reason(self):
		"""Function to make the reasoner reason concerning the ontology.

		"""
		self.utils.apply_buffered_changes()   # apply the changes in the ontology
		self.utils.sync_buffered_reasoner()   # reason upon the ontology

	def surveillance_policy(self, reachable_loc):
		""" Function to choose the next location among the reachable ones.
		It performs the following surveillance policy:
		- It mainly stays on corridors
		- If a reachable location is URGENT, it should visit it.
		
		Args:
			reachable_loc (str[]): list of reachable locations

		Returns:
			loc (str): choosen location among the reachable ones

		"""
		random.shuffle(reachable_loc)                    # shuffle the reachable location, not to have a preference between them.
		urgent_loc = self.query.ind_b2_class('URGENT')   # query for all URGENT locations
		urgent_loc = format(urgent_loc, '#', '>')        # format the URGENT locations

		log_msg = f'The urgent locations are {urgent_loc}'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		# If there are no reachable loctions.
		if not reachable_loc:
			log_msg = f'The robot can not reach any location'
			rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
			return False

		# If there is a reachable location which is URGENT, return that location.
		for loc in reachable_loc:
			if loc in urgent_loc:
				return loc

		# If there is a reachable location which is a CORRIDOR, return that location.
		for loc in reachable_loc:
			if loc in self._corridors:
				return loc

		# Return a random location (because of shuffling them).
		return reachable_loc[0]