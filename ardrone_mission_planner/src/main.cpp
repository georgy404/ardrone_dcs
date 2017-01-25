/**
*  This file is part of ardrone_dcs.
*
*  trajectory_planner planner node receive current target from mission_planner 
*  and create trajectory as array of point, send it to ardrone_pid.
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
