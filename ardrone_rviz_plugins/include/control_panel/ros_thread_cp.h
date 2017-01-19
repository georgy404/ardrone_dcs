/**
*  This file is part of ardrone_dcs.
*
*  ros_thread_cp is part of control panel rviz plugin.
*  It's provide thread for communicate with ROS system.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#pragma once
#ifndef ROSTHREADCP_H
#define ROSTHREADCP_H

#include <stdlib.h>
#include <iostream>
#include "assert.h"
#include <sys/time.h>

#include <QtCore>
#include <QThread>
#include <QString>
#include <QStringList>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ardrone_msgs/NavPose.h>
#include <ardrone_msgs/AddMarker.h>
#include <ardrone_msgs/DelMarker.h>
#include <ardrone_msgs/ClearMarkers.h>
#include <ardrone_msgs/Mode.h>
#include <ardrone_msgs/Mission.h>
#include <ardrone_autonomy/Navdata.h>


class RosThreadCP : public QObject {
	Q_OBJECT
public:
    RosThreadCP();
    virtual ~RosThreadCP();
    bool init();

    // --- Send data to ROS functions
    void SendMode(int mode);
    void SendMission(std::vector<ardrone_msgs::NavPose> mission, bool take_off, bool land);

    // --- ROS service functions
    ardrone_msgs::NavPose SendAddTager(int num);
    bool SendDeleteTarget(int num);
    bool SendClearAllTargets();
    bool SendClearTrajectory();
    void SendUpdateTarget(ardrone_msgs::NavPose target);

    // --- Drone state function
    uint GetDroneState();

private:
    ros::NodeHandle * nh;

    // --- ROS sub and pub objects
    ros::Subscriber update_marker_sub;
    ros::Subscriber navdata_sub;
    ros::Publisher update_marker_pub;
    ros::Publisher mode_pub;
    ros::Publisher mission_pub;
    ros::ServiceClient add_marker_client;
    ros::ServiceClient del_marker_client;
    ros::ServiceClient clear_markers_client;
    ros::ServiceClient clear_trajectory_client;

    // --- Thread object
    QThread * thread;

    // --- Drone state object
    uint droneState;

    // --- Callback functions
    void UpdateMarkerCallback(const ardrone_msgs::NavPose::ConstPtr &msg);
    void NavDataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg);

public slots:
    void run();

signals:
    void signalUpdateMarkerTarget(ardrone_msgs::NavPose target);
};

#endif // ROSTHREADCP_H
