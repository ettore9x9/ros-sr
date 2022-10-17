#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time

from surveillance_robot import architecture_name_mapper as anm

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_STATE_MACHINE

LIST_ACTIONS = ['complete_map','choose_next','just_visited','battery_high','battery_low']

# define state Buildmap
class Buildmap(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=LIST_ACTIONS,
                             input_keys=['buildmap_counter_in'],
                             output_keys=['buildmap_counter_out'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        time.sleep(5)
        rospy.loginfo('Executing state BUILDMAP (users = %f)'%userdata.buildmap_counter_in)
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
    rospy.init_node('surveillance_state_machine')

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
