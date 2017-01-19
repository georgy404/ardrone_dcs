/**
*  This file is part of ardrone_dcs.
*
*  ros_thread_cp is part of control panel rviz plugin.
*  It's provide thread for communicate with ROS system.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include "control_panel/ros_thread_cp.h"


RosThreadCP::RosThreadCP()
{
}

RosThreadCP::~RosThreadCP()
{
    if(ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }

    thread->wait();
}

bool RosThreadCP::init()
{
    thread = new QThread();
    this->moveToThread(thread);

    connect(thread, SIGNAL(started()), this, SLOT(run()));

    nh = new ros::NodeHandle();
    if (!ros::master::check())
        return false;

    ros::start();
    ros::Time::init();

    // Init subscribe
    update_marker_sub = nh->subscribe("markers_server/update_to_ctr_panel", 10, &RosThreadCP::UpdateMarkerCallback, this);
    navdata_sub = nh->subscribe("/ardrone/navdata", 10, &RosThreadCP::NavDataCallback, this);

    // Init publish
    update_marker_pub = nh->advertise<ardrone_msgs::NavPose>("markers_server/update_from_ctr_panel", 10);
    mode_pub = nh->advertise<ardrone_msgs::Mode>("/ardrone/mode", 10);
    mission_pub = nh->advertise<ardrone_msgs::Mission>("/ardrone/mission", 10);

    // Init services and client
    add_marker_client = nh->serviceClient<ardrone_msgs::AddMarkerRequest>("markers_server/add_marker");
    del_marker_client = nh->serviceClient<ardrone_msgs::DelMarkerRequest>("markers_server/del_marker");
    clear_markers_client = nh->serviceClient<ardrone_msgs::ClearMarkersRequest>("markers_server/clear_markers");
    clear_trajectory_client = nh->serviceClient<ardrone_msgs::ClearMarkersRequest>("markers_server/clear_trajectory");

    thread->start();

    return true;
}


// --- Send data to ROS functions

void RosThreadCP::SendMode(int mode)
{
    ardrone_msgs::Mode msg;
    msg.mode = mode;

    mode_pub.publish(msg);
}

void RosThreadCP::SendMission(std::vector<ardrone_msgs::NavPose> mission, bool take_off, bool land)
{
    if(mission.empty())
        return;

    ardrone_msgs::Mission msg;
    msg.takeoff = take_off;
    msg.land = land;
    for(int i = 0; i < mission.size(); i++) {
        ardrone_msgs::NavPose target;
        target.num = mission.at(i).num;
        target.x = mission.at(i).x;
        target.y = mission.at(i).y;
        target.z = mission.at(i).z;

        msg.mission.push_back(target);
    }

    mission_pub.publish(msg);
}


// --- Drone state function

uint RosThreadCP::GetDroneState()
{
    return droneState;
}


// --- ROS service functions

ardrone_msgs::NavPose RosThreadCP::SendAddTager(int num)
{
    ardrone_msgs::AddMarker srv;
    srv.request.num = num;

    if(add_marker_client.call(srv))
        return srv.response.marker_coords;
    else
        return ardrone_msgs::NavPose();
}

bool RosThreadCP::SendDeleteTarget(int num)
{
    ardrone_msgs::DelMarker srv;
    srv.request.num = num;

    return del_marker_client.call(srv);
}

bool RosThreadCP::SendClearAllTargets()
{
    ardrone_msgs::ClearMarkers srv;

    return clear_markers_client.call(srv);
}

bool RosThreadCP::SendClearTrajectory()
{
    ardrone_msgs::ClearMarkers srv;

    return clear_trajectory_client.call(srv);
}

void RosThreadCP::SendUpdateTarget(ardrone_msgs::NavPose target)
{
    update_marker_pub.publish(target);
}


// --- ROS subscribe functions

void RosThreadCP::UpdateMarkerCallback(const ardrone_msgs::NavPose::ConstPtr &msg)
{
    emit signalUpdateMarkerTarget(*msg);
}

void RosThreadCP::NavDataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg)
{
    droneState = msg->state;
}


// --- ROS thread function

void RosThreadCP::run()
{
    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}
