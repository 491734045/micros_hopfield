cmake_minimum_required(VERSION 2.8.3)
project(micros_hopfield)

## Find catkin macros and libraries 
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(   
   FILES
   Plan.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES micROS_Plan
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(micROS_Plan
#   src/${PROJECT_NAME}/micROS_Plan.cpp
# )

## Declare a cpp executable
# add_executable(micROS_Plan_node src/micROS_plan_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(micROS_Plan_node micROS_Plan_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(micROS_Plan_node
#   ${catkin_LIBRARIES}
# )
add_executable(plan_server src/plan_server.cpp)
target_link_libraries(plan_server ${catkin_LIBRARIES})
add_dependencies(plan_server micROS_Plan_gencpp)

add_executable(plan_client src/plan_client.cpp)
target_link_libraries(plan_client ${catkin_LIBRARIES})
add_dependencies(plan_client micROS_Plan_gencpp)

add_library(plan_visualization src/Frame_Visualization/plan_visualization.cpp)
target_link_libraries(plan_client plan_visualization)
target_link_libraries(plan_server plan_visualization)

#####################################################################
#Algorithm

##HopField
add_library(algoHopField src/Algorithm_HopField/algo.cpp)
target_link_libraries(plan_server algoHopField)

#PRM
add_library(algoPRM src/Algorithm_PRM/algo.cpp)
target_link_libraries(plan_server algoPRM)
add_library(ACRoutePlanner src/Algorithm_PRM/PRMLib/ACRoutePlanner.cpp)
target_link_libraries(plan_server ACRoutePlanner)
add_library(PrmMap src/Algorithm_PRM/PRMLib/PrmMap.cpp)
target_link_libraries(plan_server PrmMap)
#####################################################################

