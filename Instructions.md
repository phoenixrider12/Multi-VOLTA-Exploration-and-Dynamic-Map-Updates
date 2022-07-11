# Installation and Running Instructions

We assume that your VOLTA robots already have [this](https://github.com/botsync/volta) package running.

## Manual Multi-Robot Mapping

- Clone this repository to your workspace src directory
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/artparkindia/multibot_mapping.git
  ```
- Build the repository
  ```
  cd ~/catkin_ws
  catkin build
  ```
- The namespace for robot has been specified in `multivolta_exploration/launch/volta.launch` file with dafault value of volta1. When launching this file, you can set the namespace as per your requiremeny.
  ```
  roslaunch multivolta_exploration volta.launch
  ```
- After launching this file, VOLTA's Bringup, Sensors and Gmapping will be started. Now you have to start map merger node. Before that, set the correct initial poses in `map_merge/launch/map_merge.launch`. (You can set one robot as origin and measure others' relative pose.)
  ```
  roslaunch multirobot_map_merge map_merge.launch
  ```
- Now you can use joysticks to teleoperate your robot. If in case you want to use keyboard instead of joystick, set the keyboard argument to True in `volta.launch` file.


## Multi-Robot Exploration

- For running exploration, uncomment the `move_base.launch` file from `volta.launch` so that move_base node also starts working.
  ```
  roslaunch multivolta_exploration volta.launch
  ```
- Launch exploration. Default exploration file is for 2 robots, if you are using more robots, you can modify it accordingly.
  ```
  roslaunch rrt_exploration two_voltas.launch
  ```
- In your rviz, you have to select five points in the specified order as shown below to start exploration.

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/76533398/177487250-f9ff0f30-343c-4875-8f4a-549e7f5f1ea0.png">
</p>


## Dynamic Map Updates

- Before running dynamic map updates, do following two things:
  - Install Mediapipe's Objectron module from [here](https://google.github.io/mediapipe/getting_started/python.html).
  - Add `{name: simple_layer, type: "simple_layer_namespace::SimpleLayer"}` under plugins in `volta_navigation/config/costmap_global.yaml`.
  
- Launch VOLTA's Bringup
  ```
  roslaunch volta_base bringup.launch
  ```
- Launch VOLTA's Sensors
  ```
  roslaunch volta_base sensors.launch
  ```
- Launch Navigation
  ```
  roslaunch volta_navigation navigation.launch
  ```
  (You may need to change the map file defined in navigation launch file if you are using a different map)
- Launch Objectron node and you will see global costmap getting updated with obstacles (both marking and clearing).
  ```
  rosrun map_update main.py
  ```
- If you want to update your static map also with obstacles, run the following:
  ```
  rosrun map_update map_update.py
  ```
