# ardrone_dcs

Ardrone Distributed Control System (DCS) is ROS package for control Parrot AR.Drone 2.0. Package can be used to control real drone or applied together with tum_simulator (http://wiki.ros.org/tum_simulator).

This package contains:
 - ardrone autonomy pack. It's update for ardrone_autonomy package (http://wiki.ros.org/ardrone_autonomy). ardrone_autonomy was changed because odometry was misinterpreted in comparison with the ROS coordinate system. Replace files in ardrone_autonomy package.
 - ardrone pid controller. It's node for going from point to point. This node based on tum_ardrone package (http://wiki.ros.org/tum_ardrone). 
 - ardrone mission planner. It provides control over execution of mission, switch to checkpoint, switching modes.
 - ardrone rviz plugins. It's three panels for rviz. Control panel for create and correct fly mission, info panel for display diagnostics and teleop panel for manual control of drone.
 - ardrone teleop. It's a simple console program for manually control of drone.
