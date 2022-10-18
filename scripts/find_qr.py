#!/usr/bin/env python

import rospy
import random

# Import the statements that define the environment.
from surveillance_robot import environment as env

# Import constant name defined to structure the architecture.
from surveillance_robot import architecture_name_mapper as anm

# Import the custom message Statement
from surveillance_robot.msg import Statement

# Tag for identifying logs producer.
LOG_TAG = anm.NODE_STATEMENT_PUB

# Initialise a new message that this node will publish.
# The published message is of type `Statement.msg`.
def init_msg():
	msg = Statement()
	msg.stamp = rospy.Time.now()
	return msg

def main():
	# Get parameter and initialise this node as well as its publisher.
	rospy.init_node(anm.NODE_STATEMENT_PUB, log_level=rospy.INFO)
	randomness = rospy.get_param(anm.PARAM_RANDOM_STATEMENT_ACTIVE, True)
	publisher = rospy.Publisher(anm.TOPIC_STATEMENT, Statement, queue_size=1, latch=True)

	# Log information.
	log_msg = f'Initialise node `{anm.NODE_STATEMENT_PUB}` with topic `{anm.TOPIC_STATEMENT}`.'
	rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	# Configure node based on parameters to generate statements with random delays.
	statement_timing = rospy.get_param(anm.PARAM_STATEMENT_TIME, [1.0, 5.0])
	log_msg = (f'Statements are published with a delay '
	           f'in the range of [{statement_timing[0]}, {statement_timing[1]}) seconds.')
	rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	# If statements must be choosen randomically, shuffle the list of statements.
	if randomness:
		random.shuffle(env.statement_list)

	# For every statement in the statement list.
	for stat in env.statement_list:

		# If it has not been already stated.
		if stat.already_stated == 0:

			# Generate the message to be published.
			msg = init_msg()

			# Fill the message with the proper fields.
			msg.location = stat.location
			msg.door = stat.door

			# Publish the message.
			publisher.publish(msg)
			log_msg = f'Publishing statement: `Door {msg.door} is in location {msg.location}`'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			stat.stated()

			# Wait before the next random message is published.
			delay = random.uniform(statement_timing[0], statement_timing[1])
			rospy.sleep(delay)


	endmsg = init_msg()
	publisher.publish(endmsg)
			
if __name__ == '__main__':
    main()