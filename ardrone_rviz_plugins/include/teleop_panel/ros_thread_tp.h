/**
*  This file is part of ardrone_dcs.
*
*  ros_thread_tp is part of teleop panel rviz plugin.
*  It's provide thread for communicate with ROS system.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#pragma once
#ifndef ROSTHREADTP_H
#define ROSTHREADTP_H

#include <stdlib.h>
#include <iostream>
#include "assert.h"
#include <sys/time.h>

#include <QtCore>
#include <QThread>
#include <QString>
#include <QStringList>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <ardrone_msgs/Mode.h>
#include <geometry_msgs/Twist.h>

class RosThreadTP : public QObject {
	Q_OBJECT

public:
    RosThreadTP();
    virtual ~RosThreadTP();
    bool init();

    // --- Data send functions
    void SendTakeOffCmd();
    void SendLandCmd();
    void SendResetCmd();

    // --- Data interface functions
    void SetCurCmdVel(geometry_msgs::Twist cmdVel);
    void ResetCurCmdVel();
    void UpLinearVel(float vel = 0.1);
    void DownLinearVel(float vel = 0.1);
    void UpAngularVel(float vel = 0.1);
    void DownAngularVel(float vel = 0.1);
    float GetLinearVel();
    float GetAngularVel();

private:
    ros::NodeHandle * nh;

    // --- ROS sub and pub objects
    ros::Subscriber mode_sub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher land_publisher;
    ros::Publisher take_off_publisher;
    ros::Publisher reset_publisher;

    // --- ROS subscribe functions
    void ModeCallback(const ardrone_msgs::Mode::ConstPtr &msg);

    // --- ROS publish functions
    void SendCmdVel();

    // --- Thread object
    QThread * thread;

    // --- Control value objects
    bool isActive;
    float curLinearVelValue;
    float curAngularVelValue;
    geometry_msgs::Twist curVelCmd;

public slots:
    void run();

signals:
    void working(bool work);
};

#endif // ROSTHREADTP_H
