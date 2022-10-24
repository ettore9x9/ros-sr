#!/usr/bin/env python

import threading
import random
import rospy

# Import constant name defined to structure the architecture.
from surveillance_robot import architecture_name_mapper as anm

# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_BATTERY_MANAGER

STEP = 30

def loading_bar(percent, buf_size):
    # Graphical feature that prints a loading bar that graphically
    #   rapresents the percentage of the battery recharging.

    num_chars = int((percent / (buf_size / 100)) * STEP / 100)
    print("\r[", end ="")
    for i in range(0, num_chars):
        print("#", end ="")

    for j in range(0, STEP - num_chars - 1):
        print(" ", end ="")

    print("] %d %% DONE", int(percent / (buf_size / 100)), end ="")

# The battery manager class.
class BatteryManager:

    def __init__(self):
        # Initialise this node.
        rospy.init_node(anm.NODE_BATTERY_MANAGER, log_level=rospy.INFO)
        # Initialise battery level.
        self._battery_low = False
        # Initialise randomness, if enabled.
        self._randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
        if self._randomness:
            self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [15.0, 40.0])
            log_msg = (f'Random-based battery low notification active: the battery change state (i.e., low/high) with a '
                       f'delay in the range of [{self._random_battery_time[0]}, {self._random_battery_time[1]}) seconds.')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        rospy.Service(anm.SERVICE_RECHARGING, SetBool, self.recharging)

        # Start publisher on a separate thread.
        th = threading.Thread(target=self._is_battery_low)
        th.start()

        # Log information.
        log_msg = (f'Initialise node `{anm.NODE_BATTERY_MANAGER}` that publishes on topic {anm.TOPIC_BATTERY_LOW}.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def recharging(self, request):
        response = SetBoolResponse()
        if request.data == True:
            log_msg = f'Recharging...'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

            for i in range(0, STEP):
                loading_bar(i,STEP)
                rospy.sleep(anm.RECHARGING_TIME / STEP)

            print("\n")
            log_msg = f'Robot fully charged.'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            self._battery_low = False
            response.success = True
        else:
            response.success = False
        return response

    # Publish changes of battery levels. This method runs on a separate thread.
    def _is_battery_low(self):
        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        if self._randomness:
            # Publish battery level changes randomly.
            self._random_battery_notifier(publisher)
        else:
            # Publish battery level changes through a keyboard-based interface.
            self._manual_battery_notifier(publisher)

    # Publish when the battery change state (i.e., high/low) based on a random
    # delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e., `True`: battery low, `False`: battery high.
    def _random_battery_notifier(self, publisher):
        while not rospy.is_shutdown():
            if self._battery_low == False:

                # Wait for simulate battery usage.
                delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
                rospy.sleep(delay)

                # Change battery state.
                self._battery_low = True

                # Publish battery level.
                publisher.publish(Bool(self._battery_low))

                # Log state.
                log_msg = f'Robot got low battery after {delay} seconds.'
                rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    # Allow keyboard interaction to emulate battery level changes.
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e., `True`: battery low, `False`: battery high.
    def _manual_battery_notifier(self, publisher):
        # Explain keyboard-based interaction.
        print('  # Type `Low` (`L`) to notify that the battery is low.')
        print('  # Type `cnt+C` and `Enter` to quit.')
        # Publish the default value at startup.
        publisher.publish(Bool(self._battery_low))
        # Loop to enable multiple interactions.
        while not rospy.is_shutdown():
            if self._battery_low == False:
                # Wait for the user to enter a battery state.
                user_input = input(' > ')
                user_input = user_input.lower()
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

if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    BatteryManager()
    rospy.spin()

