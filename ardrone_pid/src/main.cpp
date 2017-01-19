/**
*  This file is part of ardrone_dcs.
*
*  Ardron pid node contains pid controller.
*  This node base on tum_ardrone package <https://vision.in.tum.de/data/software/tum_ardrone>.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include "ardrone_pid.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ardrone_pid");

    std::cout.flush();
    std::cout << "Starting ardrone pid controller...\n";

    // Init and start ROS node
    ArdronePid pid_controller;
    pid_controller.UpdateControl();

    return 0;
}
