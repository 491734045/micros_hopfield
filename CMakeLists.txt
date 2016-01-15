cmake_minimum_required(VERSION 2.8.3)
project(micros_hopfield)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


add_service_files(   
   FILES
   plan.srv
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
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(plan_server src/plan_server.cpp)
target_link_libraries(plan_server ${catkin_LIBRARIES})
add_dependencies(plan_server micROS_Plan_gencpp)

add_executable(plan_client src/plan_client.cpp)
target_link_libraries(plan_client ${catkin_LIBRARIES})
add_dependencies(plan_client micROS_Plan_gencpp)

add_library(plan_visualization src/plan_visualization.cpp)
target_link_libraries(plan_client plan_visualization)
target_link_libraries(plan_server plan_visualization)


##HopField
add_library(algoHopField src/hopfield_algo.cpp)
target_link_libraries(plan_server algoHopField)


