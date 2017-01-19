/**
*  This file is part of ardrone_dcs.
*
*  marker_server is node for control interactive markers.
*  It's provide create, update and remove waypoint for ardrone mission.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include "markers_server/markers_server.h"
#include "interactive_markers/interactive_marker_server.h"


MarkersServer::MarkersServer()
{
    nh = new ros::NodeHandle();

    server = new interactive_markers::InteractiveMarkerServer("markers_server");

    // Init line markers
    targets_line = new LineStrip(nh, CreateColor(0.2, 0.2, 1.0, 1.0), "targets_line"); // blue line
    path_line = new LineStrip(nh, CreateColor(1.0, 0.2, 0.2, 1.0), "path_line");       // red line

    // Init ROS subscribe functions
    update_marker_sub = nh->subscribe("markers_server/update_from_ctr_panel", 10, &MarkersServer::UpdateMarkerCallback, this);

    // Init ROS publish functions
    update_marker_pub = nh->advertise<ardrone_msgs::NavPose>("markers_server/update_to_ctr_panel", 10);

    // Init ROS service functions
    add_marker_srv = nh->advertiseService("markers_server/add_marker", &MarkersServer::AddMarker, this);
    delete_marker_srv = nh->advertiseService("markers_server/del_marker", &MarkersServer::DeleteMarker, this);
    clear_markers_srv = nh->advertiseService("markers_server/clear_markers", &MarkersServer::ClearMarkers, this);
    clear_trajectory_srv = nh->advertiseService("markers_server/clear_trajectory", &MarkersServer::ClearTarjectory, this);
    
    // Clear trajectory
    path_line->ClearAllPoints();
}


// --- ROS subscribe functions

void MarkersServer::processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
    ardrone_msgs::NavPose target;
    target.num = atoi((feedback->marker_name.c_str()));
    target.x = feedback->pose.position.x;
    target.y = feedback->pose.position.y;
    target.z = feedback->pose.position.z;

    if(CheckTarget(target))
        UpdateBox(target);

    update_marker_pub.publish(target);
}

void MarkersServer::UpdateMarkerCallback(const ardrone_msgs::NavPose::ConstPtr &msg)
{
    UpdateBox(*msg);
}


// --- ROS service functions

bool MarkersServer::AddMarker(ardrone_msgs::AddMarker::Request &req, ardrone_msgs::AddMarker::Response &res)
{
    // Pose of new marker
    ardrone_msgs::NavPose markerPose = ardrone_msgs::NavPose();
    markerPose.z = 1.0;

    // Get last old marker
    InteractiveMarker befMarker;
    if(server->get(to_string(req.num-1), befMarker)) {
        markerPose.x = befMarker.pose.position.x + 0.5;
        markerPose.y = befMarker.pose.position.y;
        markerPose.z = befMarker.pose.position.z;
    }
    markerPose.num = req.num;

    // Add targets line point
    targets_line->AddPoint(markerPose);

    // Create new marker and set pose
    InteractiveMarker newMarker = MakeMarker(to_string(markerPose.num));
    server->insert(newMarker, boost::bind(&MarkersServer::processFeedback, this, _1));
    UpdateBox(markerPose);

    // Return pose of new marker
    res.marker_coords = markerPose;

    return true;
}

bool MarkersServer::DeleteMarker(ardrone_msgs::DelMarker::Request &req, ardrone_msgs::DelMarker::Response &res)
{
    res.is_ok = false;
    if(server->erase(to_string(req.num)))
        res.is_ok = true;

    // Delete targets line point
    targets_line->DeletePoint(req.num);

    server->applyChanges();

    return true;
}

bool MarkersServer::ClearMarkers(ardrone_msgs::ClearMarkers::Request &req, ardrone_msgs::ClearMarkers::Response &res)
{
    // Clear all line points
    targets_line->ClearAllPoints();

    server->clear();
    server->applyChanges();

    res.is_ok = true;

    return true;
}

bool MarkersServer::ClearTarjectory(ardrone_msgs::ClearMarkers::Request &req, ardrone_msgs::ClearMarkers::Response &res)
{
    // Clear all path points
    path_line->ClearAllPoints();

    res.is_ok = true;

    return true;
}


// --- ROS listen TF functions for draw ardrone path line

void MarkersServer::ListenTFTranform()
{
    ardrone_msgs::NavPose point;
    point.num = -1;

    tf::StampedTransform transform;
    try {
        tf_pose_listener.lookupTransform("odom", "base_link", ros::Time(0), transform);

        // get pose
        point.x = transform.getOrigin().x();
        point.y = transform.getOrigin().y();
        point.z = transform.getOrigin().z();
    }
    catch (tf::TransformException ex) {
        std::cerr.flush();
        std::cerr << ex.what() << std::endl;
        ros::Duration(1.0).sleep();
    }

    path_line->AddPoint(point);
}


// --- Markers interface functions

Marker MarkersServer::MakeBox(InteractiveMarker &msg)
{
    Marker marker;

    marker.type = Marker::SPHERE;
    marker.scale.x = msg.scale;
    marker.scale.y = msg.scale;
    marker.scale.z = msg.scale;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl& MarkersServer::MakeBoxControl(InteractiveMarker &msg)
{
    InteractiveMarkerControl control;

    control.always_visible = true;
    control.markers.push_back(MakeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
}

InteractiveMarker MarkersServer::MakeMarker(std::string name)
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "odom";
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = name;
    int_marker.description = name;
    int_marker.scale = 0.3;

    // insert a box
    MakeBoxControl(int_marker);

    InteractiveMarkerControl control;

    // X-axes control
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // Y-axes control
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // Z-axes control
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    return int_marker;
}

void MarkersServer::UpdateBox(ardrone_msgs::NavPose target)
{
    // Update targets line point
    targets_line->UpdatePoint(target);

    geometry_msgs::Pose pose;
    pose.position.x = target.x;
    pose.position.y = target.y;
    pose.position.z = target.z;

    server->setPose(to_string(target.num), pose);
    server->applyChanges();
}

void MarkersServer::UpdateLines()
{
    targets_line->SendLine();
    path_line->SendLine();
}

std_msgs::ColorRGBA MarkersServer::CreateColor(float r, float g, float b, float a)
{
    std_msgs::ColorRGBA color;
    color.r = r; color.g = g; color.b = b; color.a = a;

    return color;
}


// --- Check data functions

bool MarkersServer::CheckTarget(ardrone_msgs::NavPose &target)
{
    return ( CheckValue(target.x, -100, 100) ||
             CheckValue(target.y, -100, 100) ||
             CheckValue(target.z,    0, 100)    );
}

bool MarkersServer::CheckValue(double &value, double min, double max)
{
    if(value < min) {
        value = min;
        return true;
    }
    if(value > max) {
        value = max;
        return true;
    }

    return false;
}



// --- Main function

int main(int argc, char** argv)
{
    ros::init(argc, argv, "markers_server_node");

    MarkersServer markers_server;

    ros::Rate rate(10);

    while (ros::ok())
    {
      markers_server.ListenTFTranform();
      markers_server.UpdateLines();

      ros::spinOnce();
      rate.sleep();
    }
}


