#! /usr/bin/env python
"""
.. module:: controller
  :platform: Unix 
  :synopsis: Python module for the node that controls the robot's trajectory
.. moduleauthor:: Ettore Sani 5322242@studenti.unige.it

This module simulates the controller node of the architecture. It implements a service that, provided the waypoints, controls the robot trajectory passing through them.

Service:
  /motion/controller

"""

### IMPORTS ###
import random
import rospy

from surveillance_robot import architecture_name_mapper as anm

from actionlib import SimpleActionServer
from surveillance_robot.msg import ControlAction, ControlFeedback, ControlResult

### GLOBAL ###
LOG_TAG = anm.NODE_CONTROLLER   # Tag for identifying logs producer

### CLASSES ###
class ControllingAction(object):
    """This class implements an action server to simulate motion controlling.
    Given a plan as a set of via points, it simulate the movements to reach each point with a random delay.

    """
    def __init__(self):
        # Get random-based parameters used by this server
        self._random_motion_time = rospy.get_param(anm.PARAM_CONTROLLER_TIME, [0.1, 2.0])
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

        log_msg = (f'`{anm.ACTION_CONTROLLER}` Action Server initialised. It will navigate trough the plan with a delay ' 
                   f'between each via point spanning in [{self._random_motion_time[0]}, {self._random_motion_time[1]}).')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


    def execute_callback(self, goal):
        """Callback function invoked when a client set a goal to the `controller` server.
        This function requires a list of via points (i.e., the plan), and it simulate a movement through each 
        point with a delay spanning in ['self._random_motion_time[0]`, `self._random_motion_time[1]`).

        """
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return
            # Wait before to reach the following via point. This is just for testing purposes.
            delay = random.uniform(self._random_motion_time[0], self._random_motion_time[1])
            rospy.sleep(delay)
            # Publish a feedback to the client to simulate that a via point has been reached. 
            feedback.reached_point = point
            self._as.publish_feedback(feedback)

            log_msg = f'Reaching point ({point.x}, {point.y}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # succeeded

def main():
    """This function initializes the controller ros node.

    """
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()

### MAIN ###
if __name__ == '__main__':
    main()