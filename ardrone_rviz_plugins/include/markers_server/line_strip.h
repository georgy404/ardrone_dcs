/**
*  This file is part of ardrone_dcs.
*
*  line_strip is node for control markers line.
*  It's using for draw trajectory and connection line of waypoints.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#pragma once
#ifndef LINESTRIP_H
#define LINESTRIP_H

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <ardrone_msgs/NavPose.h>


using namespace visualization_msgs;

class LineStrip
{
public:
    LineStrip(ros::NodeHandle *_nh, std_msgs::ColorRGBA color, std::string name = "line");

    // --- Line strip interface functions
    void AddPoint(ardrone_msgs::NavPose point);
    void UpdatePoint(ardrone_msgs::NavPose point);
    void DeletePoint(int num);
    void ClearAllPoints();

    void SendLine();


private:
    ros::NodeHandle * nh;
    ros::Publisher marker_pub;

    Marker line_marker;
    
    void CreateLine(std_msgs::ColorRGBA color, std::string name);
};


#endif // LINESTRIP_H
