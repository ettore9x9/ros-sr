# Surveillance robot #
**A ROS-based simulation of a surveillance robot in a close environment.**  
Author: *Ettore Sani*

---

## Introduction ##

This repository contains ROS-based software that simulates a surveillance robot.

[Here](https://ettore9x9.github.io/surveillance_robot/), you can find the documentation for this repository.

It has been developed for the first assignment of the Experimental Laboratory class at the University of Genoa.

In particular, the software developed uses a [Smach](http://wiki.ros.org/smach) state machine and builds an ontology with [armor](https://github.com/EmaroLab/armor), using the [armor_py_api](https://github.com/EmaroLab/armor_py_api).
The robot can store sentences provided by an external node about the environment in the ontology and call the reasoner to retrieve information; it also implements a surveillance policy for visiting the environment.
The whole software is provided in Python 3.

Moreover, the architecture is developed using a random-based approach to test each component.

## Scenario ##

The scenario involves a surveillance robot operating in an indoor environment.
The behavior of the robot is divided into two phases and can be summarized:
 1. Phase 1:
    - The robot starts in the E location of the environment.
    - The robot waits for information about the environment.
    - The robot builds the toplogical map of the environment.
 2. Phase 2:
    - The robot moves through locations following a surveillance policy.
    - For moving to a new location, the robot must plan a path and control its position.
    - When the battery is low, the robot goes to the E location and waits for recharging.
    - If a room has not been visited for a certain time, it becomes urgent.

### Environment ###

The indoor environment is composed of locations and doors:
 - E is the starting and recharging location.
 - A location with only one door is a room.
 - A location with more than one door is a corridor.
 - If two locations have the same door, the robot can move from one to the other.

### Policy ###

The robot must follow a surveillance policy; the implemented one is the following:
 - If there is an urgent location among the reachable ones, it should visit it.
 - It should mainly stay in corridors.

The surveillance policy is defined in the *surveillance_policy* method of the *helper* class defined in the script [state_machine_helper.py](utilities/surveillance_robot/state_machine_helper.py).

This is a sketch of the implementation:
```python
random.shuffle(reachable_loc)   # shuffle the reachable location, not to have a preference between them.

    # If there is a reachabe location that is URGENT, return that location.
    for loc in reachable_loc:
      if loc in urgent_loc:
        return loc

    # If there is a reachable location that is a CORRIDOR, return that location.
    for loc in reachable_loc:
      if loc in corridors:
        return loc

    # Return a randomic location (because of shuffling them).
    return reachable_loc[0]
```

### Assumptions ###

The whole scenario has the following assumptions:
 - The robot moves in a 2D environment.
 - The environment has no obstacles.
 - The environment does not change in time.
 - The robot generates a trajectory as a list of via points to follow, given its current and target positions. Following this list of via points, the robot can always reach the goal position.
 - The battery can become low at any time.
 - Even if the battery is low, the robot goes to the recharging location only if it is reachable. Otherwise, it continues traveling into the environment.
 - If the battery goes down while building the map, the robot finish to build it and then recharges.
 - The starting and the recharging positions can be different.

## Project Structure

### Package List

This repository contains a ROS package named `surveillance_robot` that includes the following resources.
 - [CMakeList.txt](CMakeList.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.
 - [setup.py](setup.py): File to `import` python modules from the `utilities` folder into the files in the `script` folder. 
 - [launch/](launch/): Contains the configuration to launch this package.
    - [manual_watch_over.launch](launcher/manual_watch_over.launch): It launches this package allowing for a keyboard-based interface.
    - [random_watch_over.launch](launcher/random_watch_over.launch): It launches this package with random-based stimulus.
 - [msg/](msg/): It contains the message exchanged through ROS topics.
    - [Statement.msg](msg/Statement.msg): It is the message representing a statement about the environment (e.g., the location 'E' has room 'D6').
    - [Point.msg](msg/Point.msg): It is the message representing a 2D point.
 - [action/](action/): It contains the definition of each action server used by this software.
    - [Plan.action](action/Plan.action): It defines the goal, feedback and results concerning 
      motion planning.
    - [Control.action](action/Control.action): It defines the goal, feedback, and results concerning motion control.
 - [scripts/](scripts/): It contains the implementation of each software component.
    - [state_machine.py](scripts/state_machine.py): Module to implement the state_machine node of the architecture, managing the transition between states.
    - [battery_manager.py](scripts/battery_manager.py): Module to implement the battery_manager node of the architecture, publishing a message when the robot has the battery low and providing a service for recharging it when at the base.
    - [find_qr.py](scripts/find_qr.py): Module to implement the find_qr node of the architecture, that simulates the acquisition of the knowledge of the surrounding environment by the robot.
    - [planner.py](scripts/planner.py): It is a dummy implementation of a motion planner.
    - [controller.py](scripts/controller.py): It is a dummy implementation of a motion 
      controller.
 - [utilities/surveillance_robot/](utilities/surveillance_robot/): It contains auxiliary python files, 
   which are exploited by the files in the `scripts` folder.
    - [architecture_name_mapper.py](utilities/surveillance_robot/architecture_name_mapper.py): Module to store the names of all nodes, topics, and services of the architecture.
    - [state_machine_helper.py](utilities/surveillance_robot/state_machine_helper.py): Module to manage subscribers and client requests for the module state_machine module through the helper class.
    - [environment.py](utilities/surveillance_robot/environment.py): Module to store the knowledge about the environment.
 - [diagrams/](diagrams/): It contains the diagrams shown below in this README file.
 - [docs/](docs/): It contains the documentation source.
 - [topological_map](topological_map/): It contains the starting topological map of the environment.

### Dependencies ###

The software dependencies are:
 - [xterm](https://xtermjs.org/docs/): to open multiple terminals with the launcher.
 - [rospy](http://wiki.ros.org/rospy): to define ROS nodes, services and related messages.
 - [roslaunch](http://wiki.ros.org/roslaunch): to launch multiple nodes.
 - [message_generation](http://wiki.ros.org/message_generation): to generate custom messages.
 - [actionlib](http://wiki.ros.org/actionlib/DetailedDescription): to define action servers.
 - [SMACH](http://wiki.ros.org/smach): to define the finite state machine.
 - [armor](https://github.com/EmaroLab/armor): to use the ontology.
 - [armor_py_api](https://github.com/EmaroLab/armor_py_api): to use the api for the armor ontology.


## Software Components

It follows the details of each software component implemented in this repository, which is available
in the `scripts/` folder.

### The `state_machine` Node ###

The `state_machine` is a node that defines the finite state machine of the architecture.
It manages the transitions between states; for the execution of each state it relies on the class `helper` of the `state_machine_helper` module.

This is a representation of the state machine architecture:

<img src="https://github.com/ettore9x9/surveillance_robot/diagrams/state_machine_diagram.png" width="900">

Where we can immediately recognize the two different phases.

As shown in the diagram, there are a couple of sub-state machine:
 - The MOVE state is a sub-state machine
 - The RECHARGE state is a sub-state machine, one of its states is an instance of the above state MOVE, which in this case is in practice a sub-sub-state machine.

### The `battery_manager` Node ###


The `battery_manager` is a node that deal with the battery of the robot and it 
implements two tasks:
 - It publishes when the robot has low battery. Depending on the modality, the battery low can be triggered randomically or by an user input.
 - It implements a service to recharge the robot. It waits for a specified recharging time, then returns the success of the operation.

For clarity purposes, the `battery_manager` node runs on a dedicated terminal; in case of manual modality, where it waits for an user input, the behavior is the following:

<img src="https://github.com/ettore9x9/surveillance_robot/diagrams/battery_manager_terminal.png" width="900">

Where the loading bar represents the robot's recharging.

With `rosparam` you might also set the `test/random_sense/active`,  `test/random_sense/battery_time` and `test/recharging_time` parameters (detailed below) to modify the behavior of the node.

### The `planner` Node ###

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/planner.png" width="900">

The `planner` node implements an action server named `motion/planner`. This is done by the 
means of the `SimpleActionServer` class based on the `Plan` action message. This action server requires the `state/get_pose/` service of the `robot-state` node and a `target` point given as the goal.

Given the current and target points, this component returns a plan as a list of `via_points`, 
which are randomly generated for simplicity. The number of `via_points` can be set with the 
`test/random_plan_points` parameter addressed below. Moreover, each `via_point` is provided 
after a delay to simulate computation, which can be tuned through the `test/random_plan_time` 
parameter. When a new `via_points` is generated, the updated plan is provided as `feedback`. When
all the `via_points` have been generated, the plan is provided as `results`.

While the picture above shows the actual implementation of the action server, you should not 
interact with it through the shown topics directly. Instead, you should use a 
[SimpleActionClient](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html), 
for instance, as:
```python
import actionlib
from arch_skeleton.msg import PlanAction, PlanGoal
...
# Initialize the client and, eventually, wait for the server.
client = actionlib.SimpleActionClient('motion/planner', PlanAction)
client.wait_for_server()
...
def feedback_callback(feedback):
    # Do something when feedback is provided.
    pass  
...
def done_callback(status, results):
    # Do something when results are provided.
    pass  
...
# Send a new `goal`, which is a message of type `PlanGoal`.
client.send_goal(goal, done_cb = done_callback, feedback_cb = feedback_callback)
...
# Get the action server state.
client.get_state()
...
# Cancel all goals (or the current goal only, i.e., `client.cancel_goal()`).
client.cancel_all_goals()
```

To observe the behavior of the `planner` you can run the following commands.
``` bash
roscore
# Open a new terminal.
rosrun arch_skeleton robot_states.py
# Open a new terminal.
rosservice call /state/set_pose "pose: { x: 0.11,  y: 0.22}"
rosparam set config/environment_size '[10,10]'
rosrun arch_skeleton planner.py
# Open a new terminal.
rosrun actionlib_tools axclient.py /motion/planner
```
Then, a GUI should appear. Set the goal you want to reach and hit the send button. Eventually, you
can cancel the goal as well. Also, you can change the `test/random_plan_points` and 
`test/random_plan_time` parameters (detailed below) to tune the behavior of the planner.

The last command of the above fragment of code requires the `actionlib-tools` package, which can be installed by typing:
```bash
sudo apt update
sudo apt install ros-noetic-actionlib-tools
```


### The `controller` Node ###

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/controller.png" width="900">

The `controller` node implements an action server named `motion/controller`. This is done by 
the means of the `SimpleActionServer` class based on the `Control` action message. This action 
server requires the `state/set_pose/` service of the `robot-state` node and a plan given as a 
list of `via_points` by the `planner`.

Given the plan and the current robot position, this component iterates for each planned 
`via_point` and waits to simulate the time spent moving the robot to that location. The 
waiting time can be tuned through the `test/random_motion_time` parameter detailed below. Each 
time a `via_point` is reached the `state/set_pose` service is invoked, and `feedback` is 
provided. When the last `via_point` is reached, the action service provides a result by 
propagating the current robot position, which has been already updated through the 
`state/set_pose` service.

Similarly to the `planner` above, instead of using the raw topics, you can rely on a 
`SimpleActionClient`, which should be instantiated as:
```python
client = actionlib.SimpleActionClient('motion/controller', ControlAction)
```
This client would accept goals of type `ControlGoal`.

To observe the behavior of the `controller` you can run the following commands.
``` bash
roscore
# Open a new terminal.
rosrun arch_skeleton robot_states.py
# Open a new terminal.
rosservice call /state/set_pose "pose: { x: 0.11,  y: 0.22}"
#rosparam set config/environment_size '[10,10]'
rosrun arch_skeleton controller.py
# Open a new terminal.
rosrun actionlib_tools axclient.py /motion/controller
```
Then, the same GUI seen for the `planner` should appear. In this case, you can test goals 
formatted as:
```yaml
via_points: 
  - 
    x: 0.109999999404
    y: 0.219999998808
  - 
    x: 3.61638021469
    y: 5.05489301682
  - 
    x: 0.292526483536
    y: 6.59786701202
  - 
    x: 4.33828830719
    y: 7.73262834549
  - 
    x: 6.0
    y: 6.0
```
You can also change the `test/random_motion_time` parameter (detailed below) to tune
the behavior of the controller.

### The `find_qr` Node ###

## Utilities components ##

It follows the details of each utility component implemented in this repository, which is available
in the `utilities/surveillance_robot` folder.

### The `state_machine_helper` Module ###


### The `architecture_name_mapper` Module ###


### The `environment` Module ###


### Launching the Software

This software has been based on ROS Noetic, and it has been developed with this Docker-based
[environment](https://hub.docker.com/repository/docker/carms84/exproblab), which already 
provides the required dependencies listed above. 

### Installation

Follow these steps to install the software.
 - Clone this repository inside your ROS workspace (which should be sourced in your `.bashrc`).
 - Run `chmod +x <file_name>` for each file inside the `scripts` folder.
 - Run `catkin_make` from the root of your ROS workspace.
 - Install `xterm` by entering the command `sudo apt install -y xterm`.

### Launchers

Use the following command to launch the software with a keyboard-based interface for speech, 
gesture, and battery levels.
```bash
roslaunch arch_skeleton manual_sense.launch
```

Use the following command to launch the software with randomized stimulus, 
i.e., speech, gesture, and battery level.
```bash
roslaunch arch_skeleton random_sense.launch
```

### ROS Parameters

This software requires the following ROS parameters.
 
 - `config/environment_size`: It represents the environment boundaries as a list of two float
   numbers, i.e., `[x_max, y_max]`. The environment will have the `x`-th coordinate spanning
   in the interval `[0, x_max)`, while the `y`-th coordinate in `[0, y_max)`.

 - `config/user_pose`: It represents the user's position as a list of two float numbers,
   i.e., `[x, y]`. This pose should be within the `environmet_size`.

 - `config/speech_commands`: It defines the keywords that the user can say to start and end
   the interaction. It must be a list made of two strings (e.g., `["Hello", "Bye"]`) that define
   the keyword to start and end the interaction, respectively.

 - `state/initial_pose`: It represents the initial robot pose as a list of two float numbers, 
   i.e., `[x, y]`. This pose should be within the `environmet_size`.

 - `test/random_plan_points`: It represents the number of via points in a plan, and it should be
   a list of two integer numbers `[min_n, max_n]`. A random value within such an interval will be
   chosen to simulate plans of different lengths.

 - `test/random_plan_time`: It represents the time required to compute the next via point of the 
   plan, and it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. 
   A random value within such an interval will be chosen to simulate the time required to 
   compute the next via points.

 - `test/random_motion_time`: It represents the time required to reach the next via point, and 
   it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. A random
   value within such an interval will be chosen to simulate the time required to reach the next 
   via points. 

 - `test/random_sense/active`: It is a boolean value that activates (i.e., `True`) or 
   deactivates (`False`) the random-based generation of stimulus (i.e., speech, gesture, and 
   battery level). If this parameter is `True`, then the three parameters below are also 
   required.  If it is `False`, then the three parameters below are not used.
 

In addition, the `random_sense.launch` also requires the following three parameters. This occurs because `test/random_sense/active` has been set to `True`.

 - `test/random_sense/gesture_time`: It indicates the time passed within two randomly generated 
   pointing gestures. It should be a list of two float numbers, i.e., `[min_time, max_time]` in 
   seconds, and the time passed between gestures will be a random value within such an interval.

 - `test/random_sense/speech_time`: It indicates the time passed within two randomly generated
   commands based on speech. It should be a list of two float numbers, i.e., 
   `[min_time, max_time]` in seconds, and the time passed between speech-based commands will be 
   a random value within such an interval.

 - `test/random_sense/battery_time`: It indicates the time passed within battery state changes 
   (i.e., low/high). It should be a list of two float numbers, i.e., `[min_time, max_time]` in 
   seconds and the time passed between changes in battery levels will be a random value within 
   such an interval.
   
---