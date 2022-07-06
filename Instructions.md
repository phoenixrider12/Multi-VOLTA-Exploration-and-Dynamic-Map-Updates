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
- In your rviz, you have to select five points as shown below to start exploration.

![rrt_rviz](https://user-images.githubusercontent.com/76533398/177487250-f9ff0f30-343c-4875-8f4a-549e7f5f1ea0.png)
