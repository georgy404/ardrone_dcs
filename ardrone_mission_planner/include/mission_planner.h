/**
*  This file is part of ardrone_dcs.
*
*  Mission planner node receive mission (list of targets) from rviz 
*  and controls execution of mission, switch command, 
*  send single commands and control mode of ardrone.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
*/

#pragma once
#ifndef __MISSIONPLANNER_H
#define __MISSIONPLANNER_H

#include <stack>

#include "ros/ros.h"
#include "ardrone_msgs/NavPose.h"
#include "ardrone_msgs/Mission.h"
#include "ardrone_msgs/Mode.h"
#include "std_msgs/Empty.h"
#include "tf/transform_listener.h"


class MissionPlanner
{
public:
    MissionPlanner();
    ~MissionPlanner();

    // --- Nodehandle for ROS magic
    ros::NodeHandle * nh;

    // ---ROS sub and pub objects
    ros::Subscriber mode_sub;
    ros::Subscriber mission_sub;

    ros::Publisher target_pub;
    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;

    tf::TransformListener drone_pose_listener;


    // --- ROS subscribe and publish functions
    void ModeCallback(const ardrone_msgs::Mode::ConstPtr &msg);
    void MissionCallback(const ardrone_msgs::Mission::ConstPtr &msg);
    void ListenTfTransform();

    
    // --- Control functions
    void RosSpinLoop();
    void RunMission();
    bool CheckTakeoff();
    bool CheckLand();

private:
//    std::stack<geometry_msgs::PoseStamped> mission;
    std::stack<ardrone_msgs::NavPose> mission;
    ardrone_msgs::NavPose drone_pose;
    int mode;
    bool b_takeoff;
    bool b_land;
    bool b_is_executable;
};


#endif /* __ARDRONE_PID_H */
