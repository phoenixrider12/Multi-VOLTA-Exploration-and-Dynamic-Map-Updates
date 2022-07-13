# Task
In this project, we are trying to develop a Multi-Robot Mapping pipeline that can be used for manual/autonomous mapping of unknown terrains with multiple robots without any prior information about the environment. Also, once the map is prepared, we are trying to incorporate dynamic environment changes in our generated occupancy grid from knowledge gained from camera sensors.

# Manual Multi-Robot Mapping

For manual multi-robot mapping, we are using standard LIDAR based SLAM Gmapping for individual robot to prepare individual maps and then merging them to produce a global map. For merging, a feature matching algorithm is used which detects overlapping features in individual maps and combines them  with/without knowing the initial position of any robot. More details can be found [here](http://wiki.ros.org/multirobot_map_merge).
<br><br>

![map_merge](https://user-images.githubusercontent.com/76533398/175873785-af5f86c0-0f01-4982-a2ae-c57ba5286568.gif)

Map Merging

## Demonstration on VOLTA Robots

https://user-images.githubusercontent.com/76533398/175872786-a48d55e6-5500-4827-b06c-3d64f1a718c8.mp4

# Multi-Robot Exploration
Exploration is always preferred while mapping because the robots can autonomously generate a map without any human effort. The classical way of exploration is to get frontier points on edges of generated occupancy grid and forwarding those points as goal to the robot so that it can traverse to the point and explore that area. There are several open-source ROS packages available for this approach, like [explore_lite](http://wiki.ros.org/explore_lite) and [frontier exploration](http://wiki.ros.org/frontier_exploration).

In our approach, we are using **RRT (Rapidly Exploring Random Trees) Exploration**. Here, modified RRT algorithm is used for detecting frontier points, which has proven to be much faster than standard exploration. In this algorithm, we run two different ROS nodes for exploration:
 - Global Frontier Detector: Find frontier points in global occupancy grid
 - Local Frontier Detector : Find frontier points in each robot's local occupancy grid.(local node for each robot)
<br>
This algorithm runs until the loop closure condition in global map is not satisfied. Here, we assume that the surface of the environment always forms a closed loop and the algorithm stops once it can’t find any open frontiers in the current map, denoting that all the traversable parts of the terrain are mapped, and no more global frontiers are found.
More details about this ROS package can be found on http://wiki.ros.org/rrt_exploration.
<br><br>

![rrt_exploration](https://user-images.githubusercontent.com/76533398/175874679-873ce59c-9f50-43f5-90ab-d7fa762a03ee.gif)

RRT Exploration

# Dynamic Map Updates
Ground robots are very commonly used in warehouses and factories where the environment is never static and it keeps changing. Hence, we want to create a system which can dynamically change the global costmap whenever there is any change in the environment.
<br><br>
For this purpose, we are creating an **Object Detection and Map Update** pipeline.
<!-- - The object detection is a 3D Object Detection module will constantly keeps checking the position of all major objects in the environment. We have used 3D object detection because for map update, we also need an estimate of the size and dimensions of the object so that we can properly inflate it in the map, and 2D object detection can't provide us that because it returns a 2D bounding box, whereas 3D object detection returns a 3D bounding box.
- Whenever it encounters any change in position of any object, it will notify the map update module. The map update module will get the initial and final position of that object and an estimate about its dimensions, and will make changes to both the positions in map. It will clear the initial position of that object and will publish its new location as a lethal obstacle so that it can get reflected in our map. -->

## 3D Object Detection

Here we are using 3D Object Detection instead of standard 2D object detection because for accurately updating costmap, we need object's position as well as an estimate of its dimensions in the real world, and 2D object detection can't provide us that information because it returns a 2D bounding box, whereas 3D object detection can provide that information because it returns a 3D bounding box in camera'a view frame.

For our purpose, we are using Mediapipe's [Objectron](https://google.github.io/mediapipe/solutions/objectron.html#camera-coordinate) module. It is an opensource module which can detect objects like chairs, shoes, coffee mugs and cameras.
![objectron](https://user-images.githubusercontent.com/76533398/178235387-539dfcab-b17a-4c49-bdf2-74c6f62dc29e.png)

After getting coordinates of all 9 points in camera view frame, we transform them into real world using [Look-At Transformation](https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/lookat-function) technique. Since we can't transform a point to its actual real world coordinate but instead to a 3D line in real world, we compute multiple such lines using multiple cameras and then use gradient descent optimization algorithm to extract the actual coordinates of each point. Then from the real world coordinates of all 9 points, we compute its center's coordinates and its dimensions.

## Map Update

For performing map updates, we have added a new layer to the global costmap as a plugin. That plugin receives data from objectron and updates the cost of object's new location as well as its previous location. Here is a demo video:

https://user-images.githubusercontent.com/76533398/178238455-f191a896-f6a8-403a-8fae-6bc10677354f.mp4

<br>
Here is a demo of VOLTA's navigation test while performing map updates:

https://user-images.githubusercontent.com/76533398/178238942-88896295-8299-405e-a5d8-0db93965d7fe.mp4

# Package Description

- multivolta_exploration: Containes all the launch files for launching VOLTA with namespaces for easy bringup of VOLTA with any custom namespace.
- map_merge: Map merging package which merges all individual maps and produce combined map.
- rrt_exploration: Modified RRT Exploration for VOLTA robot for multi-robot exploration.
- simple_layers: Package containing plugin for updating global costmap.
- map_update: Package containing scripts for running objectron and map updates.

### To install and run this package, follow these [instructions](https://github.com/phoenixrider12/multivolta_mapping/blob/main/Instructions.md).
