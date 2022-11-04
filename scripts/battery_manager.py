#!/usr/bin/env python
"""
.. module:: battery_manager
  :platform: Unix 
  :synopsis: Python module for the battery manager node
.. moduleauthor:: Ettore Sani 5322242@studenti.unige.it

This module implements the battery_manager node of the architecture, publishing a message when the robot has low battery and providing a service for recharging it when at the base.

Publishes to:
  /state/battery_low

Service:
  /state/recharging

"""

### IMPORTS ###
import threading
import random
import rospy

from surveillance_robot import architecture_name_mapper as anm

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

### GLOBAL ###
LOG_TAG = anm.NODE_BATTERY_MANAGER   # tag for identifying logs producer
STEP = 30                            # number of printing during the recharging operation

### FUNCTIONS ###
def loading_bar(percent, size):
    """Function to print a loading bar.
    Graphical feature that prints a loading bar that rapresents the percentage of the battery recharging.

    Args:
       percent (float): Actual state of recharging.
       size (float): Size of total recharging.

    """
    num_chars = int((percent / (size / 100)) * STEP / 100)   # number of 'black' cells of the loading bar
    print("\r[", end ="")
    for i in range(0, num_chars):
        print("#", end ="")
    for j in range(0, STEP - num_chars - 1):
        print(" ", end ="")
    print("] %d %% DONE", int(percent / (size / 100)), end ="")

### CLASSES ###
class BatteryManager:
    """This class is the battery manager of the robot. It publishes when the battery of the robot is low,
    and provides a service to recharge it.

    """
    def __init__(self):
        self._battery_low = False                                           # initialise the flag of the battery low
        self._recharging_time = rospy.get_param(anm.PARAM_RECHARGING_TIME, 8)   # initialize the time needed for recharging
            
        self._randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)   # initialise randomness, if enabled
        if self._randomness:
            self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [20.0, 40.0])   # initialize the range of battery duration in time
            
            log_msg = (f'Random-based battery low notification active: the battery change state (i.e., low/high) with a '
                       f'delay in the range of [{self._random_battery_time[0]}, {self._random_battery_time[1]}) seconds.')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Start the service for recharging.
        rospy.Service(anm.SERVICE_RECHARGING, SetBool, self.recharging)

        # Start publisher on a separate thread.
        th = threading.Thread(target=self._is_battery_low)
        th.start()

        log_msg = (f'Initialise node `{anm.NODE_BATTERY_MANAGER}` that publishes on topic {anm.TOPIC_BATTERY_LOW}.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def recharging(self, request):
        """Service callback to recharge the robot. It waits for a specified recharging time updating the loading bar,
        then returns the success of the operation.

        """
        response = SetBoolResponse()    # initialize the service response
        if request.data == True:

            log_msg = f'Recharging...'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

            for i in range(0, STEP):
                loading_bar(i,STEP)     # print the loading bar
                rospy.sleep(self._recharging_time / STEP)
            print("\n")

            log_msg = f'Robot fully charged.'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

            self._battery_low = False   # put the battery low flag down
            response.success = True     # successful recharging
        else:
            response.success = False
        return response

    def _is_battery_low(self):
        """Publisher of the battery level. It runs on a separate thread ad publishes when the robot has low battery.
        Depending on the modality, the battery low can be triggered randomically or by an user input.

        """
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)   # define the publisher
        if self._randomness:
            self._random_battery_notifier(publisher)   # randomically the battery becames low
        else:
            self._manual_battery_notifier(publisher)   # the battery becames low after an user command

    def _random_battery_notifier(self, publisher):
        """Function to publish when the battery becames low, based on a random delay within the interval 
        [`self._random_battery_time[0]`, `self._random_battery_time[1]`). The message is published through 
        the `publisher` input parameter.

        Args:
            publisher (rospy.Publisher): publisher to the battery low topic

        """
        while not rospy.is_shutdown():
            if self._battery_low == False:                   # if the robot has full battery

                # Wait for simulate battery usage.
                delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
                rospy.sleep(delay)

                self._battery_low = True                     # change battery state.
                publisher.publish(Bool(self._battery_low))   # publish battery level.

                log_msg = f'Robot got low battery after {delay} seconds.'
                rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def _manual_battery_notifier(self, publisher):
        """Function to publish when the battery becames low, based on a user keyboard interaction. 
        The message is published through the `publisher` input parameter.

        Args:
            publisher (rospy.Publisher): publisher to the battery low topic

        """
        # Explain keyboard-based interaction.
        print('  # Type `Low` (`L`) to notify that the battery is low.')
        print('  # Type `cnt+C` and `Enter` to quit.')

        publisher.publish(Bool(self._battery_low))   # publish the default value at startup
        while not rospy.is_shutdown():               # loop to enable multiple interactions
            if self._battery_low == False:           # if the robot is fully charged
                user_input = input(' > ')            # wait for the user input
                user_input = user_input.lower()      # lowercase the string

                # Understand the entered text.
                error = False
                if user_input == 'low' or user_input == 'l':
                    self._battery_low = True
                    rospy.loginfo(anm.tag_log('Robot got low battery.', LOG_TAG))
                else:
                    # Cannot understand the entered command.
                    print('*** USER INPUT ERROR! Try again:')
                    error = True

                # Publish the massage based on the entered command.
                if not error:
                    publisher.publish(Bool(self._battery_low))

def main():
    """This function initializes the battery_manager ros node.

    """
    rospy.init_node(anm.NODE_BATTERY_MANAGER, log_level=rospy.INFO)
    BatteryManager()
    rospy.spin()


### MAIN ###
if __name__ == "__main__":
    main()