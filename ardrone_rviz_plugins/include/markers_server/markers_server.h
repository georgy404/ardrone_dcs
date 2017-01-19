/**
*  This file is part of ardrone_dcs.
*
*  marker_server is node for control interactive markers.
*  It's provide create, update and remove waypoint for ardrone mission.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#pragma once
#ifndef MARKERS_SERVER_H
#define MARKERS_SERVER_H

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <ardrone_msgs/AddMarker.h>
#include <ardrone_msgs/DelMarker.h>
#include <ardrone_msgs/ClearMarkers.h>

#include <interactive_markers/interactive_marker_server.h>
#include "include/markers_server/line_strip.h"


using namespace visualization_msgs;

class MarkersServer
{
public:
    MarkersServer();

    // --- ROS subscribe functions
    void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback);
    void UpdateMarkerCallback(const ardrone_msgs::NavPose::ConstPtr &msg);

    // --- ROS listen TF functions
    void ListenTFTranform();

    // --- ROS service functions
    bool AddMarker(ardrone_msgs::AddMarker::Request &req, ardrone_msgs::AddMarker::Response &res);
    bool DeleteMarker(ardrone_msgs::DelMarker::Request &req, ardrone_msgs::DelMarker::Response &res);
    bool ClearMarkers(ardrone_msgs::ClearMarkers::Request &req, ardrone_msgs::ClearMarkers::Response &res);
    bool ClearTarjectory(ardrone_msgs::ClearMarkers::Request &req,ardrone_msgs::ClearMarkers::Response &res);

     // --- Markers interface functions
    InteractiveMarker MakeMarker(std::string name);
    Marker MakeBox(InteractiveMarker &msg);
    InteractiveMarkerControl& MakeBoxControl(InteractiveMarker &msg);
    void UpdateBox(ardrone_msgs::NavPose target = ardrone_msgs::NavPose());
    void UpdateLines();
    std_msgs::ColorRGBA CreateColor(float r, float g, float b, float a = 1.0);

    // --- Check data functions
    bool CheckTarget(ardrone_msgs::NavPose &target);
    bool CheckValue(double &value, double min, double max);

private:
    // --- ROS objects
    ros::NodeHandle * nh;
    ros::ServiceServer add_marker_srv;
    ros::ServiceServer delete_marker_srv;
    ros::ServiceServer clear_markers_srv;
    ros::ServiceServer clear_trajectory_srv;
    ros::Subscriber update_marker_sub;
    ros::Publisher update_marker_pub;
    ros::Publisher path_pub;

    tf::TransformListener tf_pose_listener;

    // --- Line markers object
    LineStrip * targets_line;
    LineStrip * path_line;

    // --- Marker server objects
    interactive_markers::InteractiveMarkerServer * server;
};

template <typename T>
static std::string to_string(T const& value)
{
    std::stringstream sstr;
    sstr << value;

    return sstr.str();
}

#endif // MARKERS_SERVER_H
