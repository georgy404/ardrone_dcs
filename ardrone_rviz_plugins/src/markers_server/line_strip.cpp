/**
*  This file is part of ardrone_dcs.
*
*  line_strip is node for control markers line.
*  It's using for draw trajectory and connection line of waypoints.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include "markers_server/line_strip.h"


LineStrip::LineStrip(ros::NodeHandle *_nh, std_msgs::ColorRGBA color, std::string name) :
    nh(_nh)
{
    marker_pub = nh->advertise<visualization_msgs::Marker>(name, 10);

    CreateLine(color, name);
}


// --- Line strip interface functions

void LineStrip::AddPoint(ardrone_msgs::NavPose point)
{
    geometry_msgs::Point g_point;
    g_point.x = point.x; g_point.y = point.y; g_point.z = point.z;

    if(point.num == -1) {
        line_marker.points.push_back(g_point);
    }
    else { // else we have num of point
        std::vector<geometry_msgs::Point>::iterator i;
        i = line_marker.points.begin()+(point.num-1);
        line_marker.points.insert(i, g_point);
    }
}

void LineStrip::UpdatePoint(ardrone_msgs::NavPose point)
{
    geometry_msgs::Point g_point;
    g_point.x = point.x; g_point.y = point.y; g_point.z = point.z;

    line_marker.points[point.num-1] = g_point;
}

void LineStrip::DeletePoint(int num)
{
    std::vector<geometry_msgs::Point>::iterator i;
    i = line_marker.points.begin()+(num-1);

    line_marker.points.erase(i);
}

void LineStrip::ClearAllPoints()
{
    line_marker.points.clear();
}

void LineStrip::CreateLine(std_msgs::ColorRGBA color, std::string name)
{
    line_marker.header.frame_id = "odom";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = name;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;


    // Line strip markers use only the x component of scale, for the line width
    line_marker.scale.x = 0.05;
    line_marker.scale.y = 0.05;
    line_marker.scale.z = 0.05;

    line_marker.color = color;
}

void LineStrip::SendLine()
{
    line_marker.header.stamp = ros::Time::now();
    marker_pub.publish(line_marker);
}


