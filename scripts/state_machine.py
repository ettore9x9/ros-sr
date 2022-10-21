#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time

# Import constant name defined to structure the architecture.
from surveillance_robot import architecture_name_mapper as anm

# Import the helper class.
from surveillance_robot import state_machine_helper as smh

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_STATE_MACHINE

LIST_ACTIONS = ['complete_map','reach_location','just_visited','battery_high','battery_low']

helper = smh.helper()

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
        log_msg = f'Executing state BUILDMAP.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        r = rospy.Rate(10)  # 10hz
        while not helper.map_completed:
            r.sleep()

        helper.initialize_map()

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
        log_msg = f'Executing state QUERY.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        helper.reason()
        reachable_locations = helper.robot_can_reach()
        next_location = helper.choose_next_location(reachable_locations)
        helper.goal_loc = next_location

        if helper.battery_low:
            return 'battery_low'

        return 'reach_location'

# define state Move
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=LIST_ACTIONS,
                             input_keys=['move_counter_in'],
                             output_keys=['move_counter_out'])

    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking

        log_msg = f'Executing state MOVE.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        ### move to helper.goal_loc location ###
        time.sleep(2)

        helper.update_robot_position()

        return 'just_visited'

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
                                            'reach_location':'BUILDMAP',
                                            'just_visited':'BUILDMAP',
                                            'battery_high':'BUILDMAP',
                                            'battery_low':'BUILDMAP',
                                            },
                               remapping={'buildmap_counter_in':'sm_counter', 
                                          'buildmap_counter_out':'sm_counter'})

        smach.StateMachine.add('QUERY', Query(), 
                               transitions={'complete_map':'QUERY', 
                                            'reach_location':'MOVE',
                                            'just_visited':'QUERY',
                                            'battery_high':'QUERY',
                                            'battery_low':'RECHARGE',
                                            },
                               remapping={'query_counter_in':'sm_counter', 
                                          'query_counter_out':'sm_counter'})

        smach.StateMachine.add('MOVE', Move(), 
                               transitions={'complete_map':'MOVE', 
                                            'reach_location':'MOVE',
                                            'just_visited':'QUERY',
                                            'battery_high':'MOVE',
                                            'battery_low':'RECHARGE',
                                            },
                               remapping={'move_counter_in':'sm_counter', 
                                          'move_counter_out':'sm_counter'})

        smach.StateMachine.add('RECHARGE', Recharge(), 
                               transitions={'complete_map':'RECHARGE', 
                                            'reach_location':'RECHARGE',
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
