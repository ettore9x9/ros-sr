## Init project.
cmake_minimum_required(VERSION 2.8.3)
project(surveillance_robot)

## Find dependences.
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
  message_generation
  armor
)

## Uncomment if the package has a setup.py
catkin_python_setup()

## Generate messages in the 'msg' folder.
add_message_files(
  FILES
  Statement.msg
  Point.msg
)

## Generate services in the 'srv' folder.
add_service_files(
  FILES
)

## Generate actions in the 'action' folder.
add_action_files(
  FILES
  Control.action
  Plan.action
)

## Generate added messages and services with any dependencies listed here.
generate_messages(
 DEPENDENCIES
 std_msgs
 actionlib_msgs
)

## The catkin_package macro generates cmake config files for your package.
catkin_package(
   CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime # rospy std_msgs
)

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Define runnables.
catkin_install_python(
  PROGRAMS 
    scripts/state_machine.py
  DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
