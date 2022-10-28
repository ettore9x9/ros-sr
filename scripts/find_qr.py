#!/usr/bin/env python
"""
.. module:: find_qr
  :platform: Unix 
  :synopsis: Python module for the node that finds qr codes in the environment
.. moduleauthor:: Ettore Sani 5322242@studenti.unige.it

This module implements the find_qr node of the architecture, that simulates the acquisition of the knowledge of the surrounding environment by the robot. It uses the module :mod:`environment` for the knowledge of doors and locations, publishes them and exits.

Publishes to:
  /map/statement

"""

### IMPORTS ###
import rospy
import random

from surveillance_robot import environment as env
from surveillance_robot import architecture_name_mapper as anm

from surveillance_robot.msg import Statement

### GLOBAL ###
# Tag for identifying logs producer.
LOG_TAG = anm.NODE_STATEMENT_PUB

### FUNCTIONS ###
def init_msg():
    """Function to initialize a new message with the actual time stamp.
    
    Returns:
        msg (Statement): Initialized message.

    """
    msg = Statement()
    msg.stamp = rospy.Time.now()   # actual time
    return msg

### MAIN ###
def main():
	"""This function initializes the find_qr ros node. Relying on the informations stored in the
	:mod:`surveillance_robot.environment` module, it publishes one statement at a time, with a random
	delay between them in a range specified by the PARAM_STATEMENT_TIME.
	After that all statements have been pulished, it publishes an empty statement, then exits.

	"""
	rospy.init_node(anm.NODE_STATEMENT_PUB, log_level=rospy.INFO)   # initialize this node
	randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)     # set to True if randomness is active
	publisher = rospy.Publisher(anm.TOPIC_STATEMENT, Statement, queue_size=1, latch=True)   # define the publisher

	log_msg = f'Initialise node `{anm.NODE_STATEMENT_PUB}` with topic `{anm.TOPIC_STATEMENT}`.'
	rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	statement_timing = rospy.get_param(anm.PARAM_STATEMENT_TIME, [1.0, 5.0])   # delay between statements

	log_msg = (f'Statements are published with a delay '
	           f'in the range of [{statement_timing[0]}, {statement_timing[1]}) seconds.')
	rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	if randomness:                         # if statements must be choosen randomically, shuffle the list of statements
		random.shuffle(env.statement_list)
	rospy.sleep(4)                         # wait the state machine to be ready, to not loose statements
	for stat in env.statement_list:        # for every statement in the statement list
		if stat.already_stated == 0:       # if the statement has not been already stated
			msg = init_msg()           	   # initialize the message to be published
			msg.location = stat.location   # fill the message with the location
			msg.door = stat.door           # fill the message with the door
			publisher.publish(msg)         # publish the message

			log_msg = f'Publishing statement: `Door {msg.door} is in location {msg.location}`'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

			stat.stated()                  # the statement has been stated
			if randomness:
				delay = random.uniform(statement_timing[0], statement_timing[1])   # random delay
			else:
				delay = 1                                                          # 1s delay
			rospy.sleep(delay)             # simulate the searching of a new qr code by sleeping

	endmsg = init_msg()         # initialize the end empty message
	publisher.publish(endmsg)   # publish the final message

if __name__ == '__main__':
    main()