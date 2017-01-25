/**
*  This file is part of ardrone_dcs.
*
*  trajectory_planner planner node receive current target from mission_planner 
*  and create trajectory as array of point, send it to ardrone_pid.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
*/

#include "include/mission_planner.h"


MissionPlanner::MissionPlanner() :
    b_takeoff(false),
    b_land(false),
    b_is_executable(false),
    mode(-1),
    switch_dist(0.1)
{
    std::cout.flush();
    std::cout << "Ardrone mission planner was started..." << std::endl;

    // Create node
    nh = new ros::NodeHandle();

    // Set up reading mission planner parameters
    dr_function = boost::bind(&MissionPlanner::DynConfCallback, this, _1, _2);
    dr_srv.setCallback(dr_function);

    // Init ROS sub and pub
    mode_sub = nh->subscribe("/ardrone/mode", 10, &MissionPlanner::ModeCallback, this);
    mission_sub = nh->subscribe("/ardrone/mission", 10, &MissionPlanner::MissionCallback, this);

    takeoff_pub = nh->advertise<std_msgs::Empty>("/ardrone/takeoff", 10, true);
    land_pub = nh->advertise<std_msgs::Empty>("/ardrone/land", 10, true);
    target_pub = nh->advertise<ardrone_msgs::NavPose>("/ardrone/target", 10, true);
}

MissionPlanner::~MissionPlanner()
{
}


// --- Control functions

void MissionPlanner::RosSpinLoop()
{
    const int rate = 10;
    ros::Rate loop_rate(rate);

    while(nh->ok()) {
        ListenTfTransform();
        RunMission();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MissionPlanner::RunMission()
{
    if(mode != ardrone_msgs::Mode::MODE_AUTO)
        return;

    if(mission.empty()) {
	    return;	    	
	}
    
    CheckTakeoff();

    ardrone_msgs::NavPose cur_target = mission.top();
    double dist = sqrt( std::fabs( (cur_target.x-drone_pose.x)*(cur_target.x-drone_pose.x) +
								   (cur_target.y-drone_pose.y)*(cur_target.y-drone_pose.y) +
								   (cur_target.z-drone_pose.z)*(cur_target.z-drone_pose.z) ) );

    std::cout << "dist = " << dist << std::endl;
    if(dist <= switch_dist) {
        mission.pop();
        std::cout << "dist < " << switch_dist << ", go to next\n";

        if(CheckLand())
            return;
    }

    target_pub.publish(cur_target);
}

bool MissionPlanner::CheckTakeoff()
{
    if(b_takeoff) {
        takeoff_pub.publish(std_msgs::Empty());
        b_takeoff = false;
        return true;
    }

    return false;
}

bool MissionPlanner::CheckLand()
{
    if(mission.empty() && b_land) {
        land_pub.publish(std_msgs::Empty());
        b_land = false;
        return true;
    }

    return false;
}


// --- ROS subscribe functions

void MissionPlanner::ModeCallback(const ardrone_msgs::Mode::ConstPtr &msg)
{
    mode = msg->mode;
}

void MissionPlanner::MissionCallback(const ardrone_msgs::Mission::ConstPtr &msg)
{
    b_takeoff = msg->takeoff;
    b_land = msg->land;

    // Clear old mission
    while (!mission.empty()) {
        mission.pop();
    }

    // Push new mission
    for(int i = msg->mission.size()-1; i >= 0; i--) {
        mission.push(msg->mission.at(i));
    }
}

void MissionPlanner::DynConfCallback(ardrone_mission_planner::MissionPlannerParamsConfig &config, uint32_t /*level*/)
{
    std::cout << "New MP params\n";

    switch_dist = config.Switch_dist;
}

// --- TF transform listener

void MissionPlanner::ListenTfTransform()
{
    tf::StampedTransform transform;
    try {
        drone_pose_listener.lookupTransform("odom", "base_link", ros::Time(0), transform);

        // Get pose
        drone_pose.x = transform.getOrigin().x();
        drone_pose.y = transform.getOrigin().y();
        drone_pose.z = transform.getOrigin().z();

        // Get angles
        double roll, pitch, yaw;
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(roll, pitch, yaw);
        drone_pose.yaw = yaw;
    }
    catch (tf::TransformException ex) {
        std::cerr.flush();
        std::cerr << ex.what() << std::endl;
        ros::Duration(1.0).sleep();
    }
}
