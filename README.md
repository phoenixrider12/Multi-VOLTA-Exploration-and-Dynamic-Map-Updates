# Task
In this project, we are trying to develop a Multi-Robot Exploration pipeline that can be used for autonomous mapping of unknown terrains with multiple robots without any prior information about the environment. Also, once the map is prepared, we are trying to incorporate environment changes in our generated occupancy grid from knowledge gained from camera sensors.

# Exploration Pipeline
Exploration is always preferred while mapping because the robots can autonomously generate a map without any human effort. The classical way of exploration is to get frontier points on edges of generated occupancy grid and forwarding those points as goal to the robot so that it can traverse to the point and explore that area. There are several open-source ROS packages available for this approach, like [explore_lite](http://wiki.ros.org/explore_lite) and [frontier exploration](http://wiki.ros.org/frontier_exploration).
