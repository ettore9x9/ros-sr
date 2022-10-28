#!/usr/bin/env python
"""
.. module:: state_machine 
  :platform: Unix 
  :synopsis: Python module for the state machine implementation
.. moduleauthor:: Ettore Sani 5322242@studenti.unige.it

This module implements the state_machine node of the architecture, managing the transition between states.
All subscriptions and service requests are managed by the class helper of the module :mod:`state_machine_helper`, as well as the ontology and the surveillance policy.

"""

### IMPORTS ###
import rospy
import smach
import smach_ros

from surveillance_robot import architecture_name_mapper as anm
from surveillance_robot import state_machine_helper as smh

### GLOBAL ###
LOG_TAG = anm.NODE_STATE_MACHINE   # Tag for identifying logs producer.

### CLASSES ###
class Buildmap(smach.State):
    """This class defines the state Buildmap of the state machine.

    """
    def __init__(self, helper):
        smach.State.__init__(self, 
            outcomes=['complete_map','reach_location','just_visited','battery_high','battery_low'],
            input_keys=['hlp']
            )
        self.hlp = helper
        
    def execute(self, userdata):
        """Called when executing the state Buildmap.

            Returns:
                string (str): transition to the next state.

        """
        self.hlp.build_the_map()
        return 'complete_map'

class Query(smach.State):
    """This class defines the state Query of the state machine.

    """
    def __init__(self, helper):
        smach.State.__init__(self, 
            outcomes=['complete_map','reach_location','just_visited','battery_high','battery_low'],
            input_keys=['hlp']
            )
        self.hlp = helper

    def execute(self, userdata):
        """Called when executing the state Query.

            Returns:
                string (str): transition to the next state.

        """
        ans = self.hlp.query_the_ontology()
        return ans

class Planner(smach.State):
    """This class defines the state Planner of the state machine.

    """
    def __init__(self, helper):
        smach.State.__init__(self, 
            outcomes=['control','arrived'],
            input_keys=['hlp']
            )
        self.hlp = helper

    def execute(self, userdata):
        """Called when executing the state Planner.

            Returns:
                string (str): transition to the next state.

        """
        self.hlp.plan_path()
        return 'control'

class Controller(smach.State):
    """This class defines the state Controller of the state machine.

    """
    def __init__(self, helper):
        smach.State.__init__(self, 
            outcomes=['control','arrived'],
            input_keys=['hlp']
            )
        self.hlp = helper

    def execute(self, userdata):
        """Called when executing the state Controller.

            Returns:
                string (str): transition to the next state.

        """
        self.hlp.control_robot()
        return 'arrived'

class WaitFull(smach.State):
    """This class defines the state WaitFull of the state machine.

    """
    def __init__(self, helper):
        smach.State.__init__(self, 
            outcomes=['just_visited','charged'],
            input_keys=['hlp']
            )
        self.hlp = helper

    def execute(self, userdata):
        """Called when executing the state WaitFull.

            Returns:
                string (str): transition to the next state.

        """
        self.hlp.recharge()
        return 'charged'

### MAIN ###
def main():
    """This function initializes the ROS node state_machine.
    It creates a SMACH state machine and two nested sub-state machines, 
    by relying on the `smach <http://wiki.ros.org/smach/>`_ module.
    An instance of the :class:`surveillance_robot.state_machine_helper.helper` class is passed to every state.

    """
    # Get parameter and initialise this node.
    rospy.init_node(anm.NODE_STATE_MACHINE, log_level=rospy.INFO)

    # Log information.
    log_msg = f'Initialise node `{anm.NODE_STATE_MACHINE}`.'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    # Create a SMACH state machine.
    sm = smach.StateMachine(outcomes=['container_interface'])

    # Create an instance of the class helper.
    hlp = smh.helper()

    # Open the state machine and fill it with the states.
    with sm:

        smach.StateMachine.add('BUILDMAP', Buildmap(hlp), 
                               transitions={'complete_map'  :'QUERY', 
                                            'reach_location':'BUILDMAP',
                                            'just_visited'  :'BUILDMAP',
                                            'battery_high'  :'BUILDMAP',
                                            'battery_low'   :'BUILDMAP',
                                            })

        smach.StateMachine.add('QUERY', Query(hlp), 
                               transitions={'complete_map'  :'QUERY', 
                                            'reach_location':'MOVE',
                                            'just_visited'  :'QUERY',
                                            'battery_high'  :'QUERY',
                                            'battery_low'   :'RECHARGE',
                                            })

        # Create a sub state machine
        move_subsm = smach.StateMachine(outcomes=['complete_map','reach_location','just_visited','battery_high','battery_low','charged'])
        
        # Open the sub state machine and fill it with the states.
        with move_subsm:

            smach.StateMachine.add('PLANNER', Planner(hlp), 
                                   transitions={'control':'CONTROLLER',
                                                'arrived':'PLANNER',
                                                })

            smach.StateMachine.add('CONTROLLER', Controller(hlp), 
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

        # Create a sub state machine
        recharge_subsm = smach.StateMachine(outcomes=['complete_map','reach_location','just_visited','battery_high','battery_low'])

        # Open the sub state machine and fill it with the states.
        with recharge_subsm:

            # Here, the state MOVE is a sub sub state machine.
            smach.StateMachine.add('MOVE', move_subsm, 
                                   transitions={'just_visited':'WAITFULL',
                                                'charged'     :'MOVE'
                                                })

            smach.StateMachine.add('WAITFULL', WaitFull(hlp), 
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

    # Create and start the introspection server for visualization.
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine.
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application.
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()