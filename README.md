# ardrone_dcs

Ardrone Distributed Control System (DCS) is ROS package for control Parrot AR.Drone 2.0. Package can be used to control real drone or applied together with tum_simulator (http://wiki.ros.org/tum_simulator).

![screenshot from 2016-12-28 16 35 44](https://cloud.githubusercontent.com/assets/25231282/22289514/71895fa6-e315-11e6-927c-cc7830c4ca5d.png)
![screenshot from 2016-12-28 16 35 59](https://cloud.githubusercontent.com/assets/25231282/22289515/718b45b4-e315-11e6-8f72-660f711c4eca.png)

This package contains:
 - ardrone autonomy pack. It's update for ardrone_autonomy package (http://wiki.ros.org/ardrone_autonomy). ardrone_autonomy was changed because odometry was misinterpreted in comparison with the ROS coordinate system. Replace files in ardrone_autonomy package.
 - ardrone pid controller. It's node for going from point to point. This node based on tum_ardrone package (http://wiki.ros.org/tum_ardrone). 
 - ardrone mission planner. It provides control over execution of mission, switch to checkpoint, switching modes.
 - ardrone rviz plugins. It's three panels for rviz. Control panel for create and correct fly mission, info panel for display diagnostics and teleop panel for manual control of drone.
 - ardrone teleop. It's a simple terminal program for manually control of drone.
