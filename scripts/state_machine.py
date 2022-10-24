#!/usr/bin/env python

import rospy
import smach
import smach_ros

from surveillance_robot import architecture_name_mapper as anm
from surveillance_robot import state_machine_helper as smh

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_STATE_MACHINE

helper = smh.helper()

# define state Buildmap
class Buildmap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete_map','reach_location','just_visited','battery_high','battery_low'])
        
    def execute(self, userdata):
        helper.build_the_map()
        return 'complete_map'

# define state Query
class Query(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete_map','reach_location','just_visited','battery_high','battery_low'])

    def execute(self, userdata):
        ans = helper.query_the_ontology()
        return ans

# define state Planner
class Planner(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['control','arrived'])

    def execute(self, userdata):
        helper.plan_path()
        return 'control'

# define state Controller
class Controller(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['control','arrived'])

    def execute(self, userdata):
        helper.control_robot()
        return 'arrived'

# define state WaitFull
class WaitFull(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['just_visited','charged'])

    def execute(self, userdata):
        helper.recharge()
        return 'charged'

def main():
    # Get parameter and initialise this node.
    rospy.init_node(anm.NODE_STATE_MACHINE, log_level=rospy.INFO)

    # Log information.
    log_msg = f'Initialise node `{anm.NODE_STATE_MACHINE}`.'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('BUILDMAP', Buildmap(), 
                               transitions={'complete_map'  :'QUERY', 
                                            'reach_location':'BUILDMAP',
                                            'just_visited'  :'BUILDMAP',
                                            'battery_high'  :'BUILDMAP',
                                            'battery_low'   :'BUILDMAP',
                                            })

        smach.StateMachine.add('QUERY', Query(), 
                               transitions={'complete_map'  :'QUERY', 
                                            'reach_location':'MOVE',
                                            'just_visited'  :'QUERY',
                                            'battery_high'  :'QUERY',
                                            'battery_low'   :'RECHARGE',
                                            })

        move_subsm = smach.StateMachine(outcomes=['complete_map','reach_location','just_visited','battery_high','battery_low','charged'])
        
        with move_subsm:

            smach.StateMachine.add('PLANNER', Planner(), 
                                   transitions={'control':'CONTROLLER',
                                                'arrived':'PLANNER',
                                                })

            smach.StateMachine.add('CONTROLLER', Controller(), 
                                   transitions={'control':'CONTROLLER',
                                                'arrived':'just_visited',
                                                })

        smach.StateMachine.add('MOVE', move_subsm,
                               transitions={'complete_map'  :'MOVE',
                                            'reach_location':'MOVE',
                                            'just_visited'  :'QUERY',
                                            'battery_high'  :'MOVE',
                                            'battery_low'   :'RECHARGE',
                                            'charged'       :'MOVE',
                                            })

        recharge_subsm = smach.StateMachine(outcomes=['complete_map','reach_location','just_visited','battery_high','battery_low'])

        with recharge_subsm:

            smach.StateMachine.add('MOVE', move_subsm, 
                                   transitions={'just_visited':'WAITFULL',
                                                'charged'     :'MOVE'
                                                })

            smach.StateMachine.add('WAITFULL', WaitFull(), 
                                   transitions={'just_visited':'WAITFULL',
                                                'charged'     :'battery_high',
                                                })

        smach.StateMachine.add('RECHARGE', recharge_subsm,
                               transitions={'complete_map'  :'RECHARGE',
                                            'reach_location':'RECHARGE',
                                            'just_visited'  :'RECHARGE',
                                            'battery_high'  :'QUERY',
                                            'battery_low'   :'RECHARGE',
                                            })

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
