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
- After launching this file, VOLTA's Bringup, Sensors and Gmapping will be started. Now you have to start map merger node.
  ```
  roslaunch multirobot_map_merge map_merge.launch
  ```
- Now you can use joysticks to teleoperate your robot. If in case you want to use keyboard instead of joystick, set the keyboard argument to True in `volta.launch` file.
