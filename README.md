# Task
In this project, we are trying to develop a Multi-Robot Exploration pipeline that can be used for autonomous mapping of unknown terrains with multiple robots without any prior information about the environment. Also, once the map is prepared, we are trying to incorporate environment changes in our generated occupancy grid from knowledge gained from camera sensors.

# Exploration Pipeline
Exploration is always preferred while mapping because the robots can autonomously generate a map without any human effort. The classical way of exploration is to get frontier points on edges of generated occupancy grid and forwarding those points as goal to the robot so that it can traverse to the point and explore that area. There are several open-source ROS packages available for this approach, like [explore_lite](http://wiki.ros.org/explore_lite) and [frontier exploration](http://wiki.ros.org/frontier_exploration).

![](https://github.com/phoenixrider12/multivolta_mapping/blob/main/rsc/explore_lite.gif)
<br>
Explore Lite
<br><br>
![](https://github.com/phoenixrider12/multivolta_mapping/blob/main/rsc/frontier_exploration.gif)
<br>
Frontier Exploration

In our approach, we are using **RRT (Rapidly Exploring Random Trees) Exploration**. Here, modified RRT algorithm is used for detecting frontier points, which has proven to be much faster than standard exploration. In this algorithm, we run two different ROS nodes for exploration:
 - Global Frontier Detector: Find frontier points in global occupancy grid
 - Local Frontier Detector : Find frontier points in each robot's local occupancy grid.(local node for each robot)
<br>
This algorithm runs until the loop closure condition in global map is not satisfied. Here, we assume that the surface of the environment always forms a closed loop and the algorithm stops once it can’t find any open frontiers in the current map, denoting that all the traversable parts of the terrain are mapped, and no more global frontiers are found.
More details about this ROS package can be found on http://wiki.ros.org/rrt_exploration.
<br><br>
<img src = "https://github.com/phoenixrider12/multivolta_mapping/blob/main/rsc/rrt_exploration.gif">
RRT Exploration

## Map Merging
In multi-robot exploration, we need a merging algorithm which can efficiently merge the individual maps of each robot and produce a global map. For this purpose, a feature matching algorithm is used which detects overlapping features in individual maps and combines them  without knowing the initial position of any robot. More details can be found [here](http://wiki.ros.org/multirobot_map_merge)
<br><br>
![](https://github.com/phoenixrider12/multivolta_mapping/blob/main/rsc/map_merge.gif)
<br>
Map Merging
