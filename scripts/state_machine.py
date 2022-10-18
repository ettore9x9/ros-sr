#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time

# Import constant name defined to structure the architecture.
from surveillance_robot import architecture_name_mapper as anm

# Import the custom message Statement
from surveillance_robot.msg import Statement

from armor_api.armor_client import ArmorClient
from armor_api.armor_utils_client import ArmorUtilsClient

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_STATE_MACHINE

LIST_ACTIONS = ['complete_map','choose_next','just_visited','battery_high','battery_low']

map_completed = 0

client = ArmorClient('mapSurveillance', 'ontoRef')
utils = ArmorUtilsClient(client)

def Buildmap_cb(stat):
    
    if stat.location == '':
        global map_completed
        map_completed = 1
    else:
        client.call('ADD', 'OBJECTPROP', 'IND', ['hasDoor', stat.location, stat.door])
        log_msg = f'Statement `Door {stat.door} is in location {stat.location}`added to the ontology.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

# define state Buildmap
class Buildmap(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=LIST_ACTIONS,
                             input_keys=['buildmap_counter_in'],
                             output_keys=['buildmap_counter_out'])

        owl_file_path = anm.PATH_TO_PKG+'/topological_map/topological_map.owl'
        iri = 'http://bnc/exp-rob-lab/2022-23'
        utils.load_ref_from_file(owl_file_path, iri, True, 'PELLET', False)
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        subscriber = rospy.Subscriber(anm.TOPIC_STATEMENT, Statement, Buildmap_cb)

        log_msg = f'Executing state BUILDMAP (users = {userdata.buildmap_counter_in}).'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        r = rospy.Rate(10)  # 10hz
        while not map_completed:
            r.sleep()

        userdata.buildmap_counter_out = userdata.buildmap_counter_in + 1
        return 'complete_map'

# define state Query
class Query(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=LIST_ACTIONS,
                             input_keys=['query_counter_in'],
                             output_keys=['query_counter_out'])

    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        time.sleep(5)
        rospy.loginfo('Executing state QUERY (users = %f)'%userdata.query_counter_in)
        userdata.query_counter_out = userdata.query_counter_in + 1
        return 'choose_next'

# define state Move
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=LIST_ACTIONS,
                             input_keys=['move_counter_in'],
                             output_keys=['move_counter_out'])

    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        time.sleep(5)
        rospy.loginfo('Executing state MOVE (users = %f)'%userdata.move_counter_in)
        userdata.move_counter_out = userdata.move_counter_in + 1
        return 'battery_low'

# define state Recharge
class Recharge(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=LIST_ACTIONS,
                             input_keys=['recharge_counter_in'],
                             output_keys=['recharge_counter_out'])

    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        time.sleep(5)
        rospy.loginfo('Executing state RECHARGE (users = %f)'%userdata.recharge_counter_in)
        userdata.recharge_counter_out = userdata.recharge_counter_in + 1
        return 'battery_high'

        
def main():
    # Get parameter and initialise this node.
    rospy.init_node(anm.NODE_STATE_MACHINE, log_level=rospy.INFO)

    # Log information.
    log_msg = f'Initialise node `{anm.NODE_STATE_MACHINE}`.'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('BUILDMAP', Buildmap(), 
                               transitions={'complete_map':'QUERY', 
                                            'choose_next':'BUILDMAP',
                                            'just_visited':'BUILDMAP',
                                            'battery_high':'BUILDMAP',
                                            'battery_low':'BUILDMAP',
                                            },
                               remapping={'buildmap_counter_in':'sm_counter', 
                                          'buildmap_counter_out':'sm_counter'})

        smach.StateMachine.add('QUERY', Query(), 
                               transitions={'complete_map':'QUERY', 
                                            'choose_next':'MOVE',
                                            'just_visited':'QUERY',
                                            'battery_high':'QUERY',
                                            'battery_low':'RECHARGE',
                                            },
                               remapping={'query_counter_in':'sm_counter', 
                                          'query_counter_out':'sm_counter'})

        smach.StateMachine.add('MOVE', Move(), 
                               transitions={'complete_map':'MOVE', 
                                            'choose_next':'MOVE',
                                            'just_visited':'QUERY',
                                            'battery_high':'MOVE',
                                            'battery_low':'RECHARGE',
                                            },
                               remapping={'move_counter_in':'sm_counter', 
                                          'move_counter_out':'sm_counter'})

        smach.StateMachine.add('RECHARGE', Recharge(), 
                               transitions={'complete_map':'RECHARGE', 
                                            'choose_next':'RECHARGE',
                                            'just_visited':'RECHARGE',
                                            'battery_high':'QUERY',
                                            'battery_low':'RECHARGE',
                                            },
                               remapping={'recharge_counter_in':'sm_counter', 
                                          'recharge_counter_out':'sm_counter'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
