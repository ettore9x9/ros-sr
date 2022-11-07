# Surveillance robot #
**A ROS-based simulation of a surveillance robot in a close environment.**  
Author: *Ettore Sani*
mail: 5322242@studenti.unige.it

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
    - The robot builds the topological map of the environment.
 2. Phase 2:
    - The robot moves through locations following a surveillance policy.
    - For moving to a new location, the robot must plan a path and control its position.
    - When the battery is low, the robot goes to the E location and waits for recharging.
    - If a room has not been visited for some time, it becomes urgent.

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

This is a sketch of the policy implementation:
```python
random.shuffle(reachable_loc)   # shuffle the reachable location, not to have a preference between them.

    # If there is a reachable location that is URGENT, return that location.
    for loc in reachable_loc:
      if loc in urgent_loc:
        return loc

    # If there is a reachable location that is a CORRIDOR, return that location.
    for loc in reachable_loc:
      if loc in corridors:
        return loc

    # Return a random location (because of shuffling them).
    return reachable_loc[0]
```

### Assumptions ###

The whole scenario has the following assumptions:
 - The robot moves in a 2D environment.
 - The environment has no obstacles.
 - The environment does not change in time.
 - The robot generates a trajectory as a list of via points to follow, given its current and target positions; by following this list of via points, the robot can always reach the goal position.
 - The battery can become low at any time.
 - Even if the battery is low, the robot goes to the recharging location only if it is reachable; otherwise, it continues traveling into the environment.
 - If the battery goes down while building the map, the robot finishes the build and then recharges.
 - The starting and the recharging positions can be different.

## Software architecture ##

Given the scenario and the assumptions, the software architecture is developed accordingly.

### Component diagram ###

<img src="https://github.com/ettore9x9/surveillance_robot/blob/master/diagrams/component_diagram.png" width="600">

In the component diagram we can see that the `state_machine` node is the centre of the whole architecture.
Each other software component simulates a task performed by the robot, such as planning the trajectory of the robot.
The `battery_manager` component provides two interfaces with the state_machine, that are:
 - *Bool*: a message that triggers when the robot gots low battery, defined in the standard ros service library.
 - *SetBool*: a service used for recharging the robot, defined in the standard ros service library.


### Sequence diagram ###

<img src="https://github.com/ettore9x9/surveillance_robot/blob/master/diagrams/temporal_diagram.png" width="900">

The sequence diagram shows two important aspects of the architecture:
 - The `find_qr` node executes until all statements are published, then publishes an end message and exits. It simulates the phase 1 of the scenario: when the robot collects data from the environment.
 - The `battery_manager` node is always active, publishing when the robot gots low battery. It is called by the `state_machine` node for recharging the robot, implemented as a blocking service.

### States diagram ###

<img src="https://github.com/ettore9x9/surveillance_robot/blob/master/diagrams/states_diagram.png" width="900">

The states diagram is made from the point of view of the state machine, for more details on each state see the state machine diagram in the dedicated section.

The phase 2 is an infinite loop starting always with the query to the ontology to retreive the reachable locations.
After each robot's movement, the ontology is updated, taking care of the time stamps.

### ROS messages and actions ###

For building interfaces between nodes, in this package there are some custom messages and actions:
 - `Point.msg`: 2D point in space, defined by x and y coordinates.
 - `Statement.msg`: couple of a door and a location, with a timestamp. It represents a statement retreived from the environment, to be stored in the ontology.
 - `Plan.action`: motion planning interface, depending on the `Point.msg`.
   - *goal*: target and actual points.
   - *result*: list of via points.
   - *feedback*: list of via_points computed so far.
 - `Control.action`: motion controlling interface, depending on the `Point.msg`.
   - *goal*: list of via points.
   - *result*: reached point.
   - *feedback*: last via point reached so far.

## Project Structure

### Package List

This repository contains a ROS package named `surveillance_robot` that includes the following resources:
 - [CMakeList.txt](CMakeList.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.
 - [setup.py](setup.py): File to `import` python modules from the `utilities` folder into the files in the `script` folder. 
 - [launch/](launch/): Contains the configuration to launch this package.
    - [manual_watch_over.launch](launcher/manual_watch_over.launch): It launches this package allowing for a keyboard-based interface.
    - [random_watch_over.launch](launcher/random_watch_over.launch): It launches this package with random-based stimulus.
 - [msg/](msg/): It contains messages exchanged through ROS topics.
    - [Statement.msg](msg/Statement.msg): It is the message representing a statement about the environment (e.g., the location 'E' has room 'D6').
    - [Point.msg](msg/Point.msg): It is the message representing a 2D point.
 - [action/](action/): It contains the definition of each action server used by this software.
    - [Plan.action](action/Plan.action): It defines the goal, feedback and results concerning 
      motion planning.
    - [Control.action](action/Control.action): It defines the goal, feedback, and results concerning motion control.
 - [scripts/](scripts/): It contains the implementation of each software component.
    - [state_machine.py](scripts/state_machine.py): Module to implement the state_machine node of the architecture, managing the transition between states.
    - [battery_manager.py](scripts/battery_manager.py): Module to implement the battery_manager node of the architecture, publishing a message when the robot has the battery low and providing a service for recharging it when at the base.
    - [find_qr.py](scripts/find_qr.py): Module to implement the find_qr node of the architecture that simulates the acquisition of the knowledge of the surrounding environment by the robot.
    - [planner.py](scripts/planner.py): It is a dummy implementation of a motion planner.
    - [controller.py](scripts/controller.py): It is a dummy implementation of a motion 
      controller.
 - [utilities/surveillance_robot/](utilities/surveillance_robot/): It contains auxiliary python files, 
   which are exploited by the files in the `scripts` folder.
    - [architecture_name_mapper.py](utilities/surveillance_robot/architecture_name_mapper.py): Module to store the names of all nodes, topics, and services of the architecture.
    - [state_machine_helper.py](utilities/surveillance_robot/state_machine_helper.py): Module to manage subscribers and client requests for the state_machine module through the helper class.
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
It manages the transitions between states; for the execution of each state, it relies on the class `helper` of the `state_machine_helper` module.

This is a representation of the state machine architecture:

<img src="https://github.com/ettore9x9/surveillance_robot/blob/master/diagrams/state_machine_flowchart.png" width="900">

Where we can immediately recognize the two different phases.

As shown in the diagram, there is a couple of sub-state machines:
 - The MOVE state is a sub-state machine.
 - The RECHARGE state is a sub-state machine; one of its states is an instance of the state MOVE, which now is a sub-sub-state machine.

The structure of this node is simple thanks to the `state_machine_helper` module; for example, a state is defined as:

```python
class Buildmap(smach.State):
    def __init__(self, helper):
        smach.State.__init__(self, 
            outcomes=['complete_map','reach_location','just_visited','battery_high','battery_low'],
            input_keys=['hlp']
            )
        self.hlp = helper
        
    def execute(self, userdata):
        self.hlp.build_the_map()
        return 'complete_map'
```

The helper class is passed to states as an input parameter.

### The `battery_manager` Node ###


The `battery_manager` is a node that deals with the battery of the robot, and it implements two tasks:
 - It publishes when the robot has a low battery; depending on the modality, the battery low can be triggered randomically or by user input.
 - It implements a service to recharge the robot that waits for a specified recharging time, then returns success.

For clarity purposes, the `battery_manager` node runs on a dedicated terminal; in the case of manual modality, where it waits for user input, the behavior is the following:

<img src="https://github.com/ettore9x9/surveillance_robot/blob/master/diagrams/battery_manager_terminal.png" width="600">

Where the loading bar represents the robot's recharging.

With `rosparam` you might also set the `test/random_sense/active`,  `test/random_sense/battery_time`, and `test/recharging_time` parameters (detailed below) to modify the behavior of the node.

### The `planner` Node ###

The `planner` node implements an action server named `motion/planner`. 
This is done thanks to the `SimpleActionServer` class based on the `Plan` action message. 
This action server requires a `starting` and a `target` point given by the goal.

Provided the initial position, this service plans a variable number of waypoints to reach the goal position. 
For simplicity, it returns a plan as a list of `via_points` generated randomically.
The number of `via_points` can be set with the `test/random_plan_points` parameter addressed below. Moreover, each `via_point` is provided after a delay to simulate computation, which can be tuned through the `test/random_plan_time` parameter. 
When a new `via_points` is generated, the updated plan is provided as `feedback`. 
When all the `via_points` have been generated the plan is provided as `results`.

### The `controller` Node ###

The `controller` node implements an action server named `motion/controller`. 
This is done thanks to the `SimpleActionServer` class based on the `Control` action message. 
This action server requires a plan given as a list of `via_points` by the `planner`.

Given the plan, this component iterates for each planned `via_point` and waits to simulate the time spent moving the robot to that location.
The waiting time can be tuned through the `test/random_motion_time` parameter detailed below. 
Each time a `via_point` is reached, a `feedback` is provided. 
When the last `via_point` is reached, the action service provides a result by propagating the current robot position.

### The `find_qr` Node ###

The `find_qr` node implements a publisher that simulates the robot's acquisition of knowledge of the surrounding environment.

Depending if the ros parameter `test/random_sense/active` is set to true or false, it shows two different behaviors:
 - True: It uses the utility component `environment` for the knowledge of doors and locations, publishing them with a random delay in the range specified by the ros parameter `test/random_sense/statement_time`.
 - False: It asks the user to type the knowledge of the environment, publishing statements until the user exits by typing `quit`.

This image shows the behavior of the find_qr node in the case where the random publisher is selected:

<img src="https://github.com/ettore9x9/surveillance_robot/blob/master/diagrams/find_qr_terminal.png" width="600">

All the environment's statements are published on the topic `map/statement`; they are made of a location and a door. 
The knowledge of rooms and corridors is inferred by the reasoner.

## Utilities components ##

It follows the details of each utility component implemented in this repository, which is available
in the `utilities/surveillance_robot` folder.

### The `state_machine_helper` Module ###

The `state_machine_helper` module manages subscribers and client requests for the node `state_machine` through the helper class.
It provides several methods to exchange information with the ontology, thanks to the api provided by the [armor_py_api](https://github.com/EmaroLab/armor_py_api).
This module has several interfaces with other modules, such as `battery_manager`, `find_qr`, `planner`, and `controller` modules.

It implements several methods called by the `state_machine` node, such as:
 - `build_the_map`: called by the **Buildmap** state, which waits until the map is completed; then disjoints all individuals in the ontology, asks the reasoner to reason with the given ontology, and queries it for locations that are corridors.
 - `query_the_ontology`: called by the **Query** state, that asks the ontology about the locations that the robot can reach and chooses one of them according to the battery level of the robot and the surveillance policy.
 - `recharge`: called by the **Waitfull** state, which requests the service `state/recharging` that recharges the battery. This service is blocking and returns when the battery is fully charged. Then it puts down the battery_low flag.
 - `plan_path`: called by the **Planner** state, which requests the action service `motion/planner` that plan the via points between the actual and the goal positions. The goal position is chosen randomically. It waits until the path is planned, then returns.
 - `control_robot`: called by the **Controller** state, which requests the action service `motion/controller` that controls the robot through waypoints. It waits until the goal point has been reached, updates the ontology, then returns.

The `state_machine_helper` module implements two callbacks:
 - The `_Buildmap_cb` to the `map/statement` topic: is used for building the ontology. The message received contains a location coupled with a door. If the message is empty, it means that the map is completed, so it raises the map_completed flag.
 - The `_Battery_cb` to the `/state/battery_low` topic: it triggers when the robot needs to recharge or when the robot is completely charged. It changes the battery_low flag according to the received message.

### The `architecture_name_mapper` Module ###

The `architecture_name_mapper` module organizes the package: it stores the names of all nodes, topics, and services of the architecture.
It also provides information for storing the ontology and a function to print logs with the producer tag.

### The `environment` Module ###

The `environment` module stores the knowledge about the environment that the `find_qr` node uses to build the default map if it is in a random setting.

The default environment is:

<img src="https://github.com/ettore9x9/surveillance_robot/blob/master/diagrams/default_environment.png" width="600">

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

Use the following command to launch the software with a keyboard-based interface:
```bash
roslaunch surveillance_robot manual_watch_over.launch
```

Use the following command to launch the software with randomized stimulus:
```bash
roslaunch surveillance_robot random_watch_over.launch
```

### ROS Parameters

This software requires the following ROS parameters.
 - `test/random_plan_points`: It represents the number of via points in a plan, and it should be a list of two integer numbers `[min_n, max_n]`. A random value within such an interval will be chosen to simulate plans of different lengths.
 - `test/random_plan_time`: It represents the time required to compute the next via point of the plan, and it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. A random value within such an interval will be chosen to simulate the time required to compute the next via points.
 - `test/random_motion_time`: It represents the time required to reach the next via point, and it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. A random value within such an interval will be chosen to simulate the time required to reach the next via points.
 - `test/recharging_time`: It represents the time required for the robot to get fully charged.
 - `test/random_sense/active`: It is a boolean value that activates (i.e., `True`) or deactivates (`False`) the random-based stimulus' generation. If this parameter is `True` the three parameters below are also required.  If it is `False` the three parameters below are not used.
 

In addition, the `random_watch_over.launch` also requires the following parameters. This occurs because `test/random_sense/active` has been set to `True`.
 - `test/random_sense/battery_time`: It indicates the time passed within the battery state becomes low. It should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds and the time passed after the robot starts moving after a recharge will be a random value within such an interval.
 - `test/random_sense/statement_time`: It indicates the time passed between publishing two statements. It should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds, and the time passed after publishing a statement will be a random value within such an interval.
   
---