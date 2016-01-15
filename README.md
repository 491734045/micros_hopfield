# Introduction
----------------
We are pleased to announce a new package for path planning, an implementation of a specified Hopfield neural network based approach. Its source code is available now.
 
The package aims to give an optimal path for an agent to get its destination in an obstacle environment. The agent starts from a random position except the barrier area in the map, and then generates a feasible path to its destination while avoiding all the obstacles. Furthermore, it can support multiple agents to plan in the map. 
 
There are two cpp files in this package: ```Plan_client.cpp``` and ```Plan_server.cpp```. 

# System Requirements
------------------------
- Ubuntu 14.04
- ROS indigo

# Installation
------------------------
## with catkin
```
cd catkin_ws/src
git clone http://github.com/491734045/micros_hopfield
cd ..
catkin_make
```


# Quick Start
------------------------
- Setup `rviz`
```
roscore & rosrun rviz rviz
```
Set the config parameter in rviz : File-open ```config-user.rviz```

-**Open a new consol for sever**. Set up environment variables
```
source devel/setup.bash
```
Note: Run this command under `~/catkin_ws` directory.
- Run Server Node
```
rosrun micros_hopfield plan_server
```
Note: Run this command under `~/catkin_ws` directory, due to loading the terrain map.

-**Open a new consol for client**. Set up environment variables
```
source devel/setup.bash
```
Note: Run this command under `~/catkin_ws` directory.
- Run Client Node
```
rosrun micros_hopfield plan_client i
```
where *i* is the client ID, *i* = 1,2,3...

Note: Run different clients in independent terminal

- Demo
![demo_micros_hopfield](https://cloud.githubusercontent.com/assets/11674154/8541617/243de8ec-24c2-11e5-9187-e21bb6d23943.png)

- The green texture presents a terrain with different altitude.
- The blue cylinder is an obstacle.
- The red and blue lines are the pathes generated for two different agents.


Note: The package is inspired by and adapted from [1]. Related details about neural network based path planning may also be found in [2], [3] and [4].
- [1] Chonghong Fan, Youzhang Lu, Hong Liu, Shangteng Huang. Path planning for mobile robot based on neural networks. Computer Engineering and Application, 2004, 8: 86-89. (in Chinese)
- [2] Roy Glasius, Andrzej Komoda, Stan C.A.M. Gielen. Neural network dynamics for path planning and obstacle avoidance. Neural Networks, 1995, 8(1):125-133.
- [3] Simon X. Yang, Max Meng. An efficient neural network approach to dynamic robot motion planning. Neural Networks, 2000, 13(2):143-148.
- [4] Simon X. Yang, Max Meng. Neural network approaches to dynamic collision-free trajectory generation. IEEE SMC Part B, 2001, 31(3): 302-318.
