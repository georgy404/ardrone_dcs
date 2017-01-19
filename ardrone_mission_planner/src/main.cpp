/**
*  This file is part of ardrone_dcs.
*
*  Mission planner node receive mission (list of targets) from rviz 
*  and controls execution of mission, switch command, 
*  send single commands and control mode of ardrone.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
*/

#include "include/mission_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ardrone_mission_planner");

    // Init and start ROS node
    MissionPlanner mp;
    mp.RosSpinLoop();

    return 0;
}
