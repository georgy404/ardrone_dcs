/**
*  This file is part of ardrone_dcs.
*
*  ros_thread_ip is part of info panel rviz plugin.
*  It's provide thread for communicate with ROS system.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#pragma once
#ifndef ROSTHREADIP_H
#define ROSTHREADIP_H

#include <stdlib.h>
#include <iostream>
#include "assert.h"
#include <sys/time.h>

#include <QtCore>
#include <QThread>
#include <QString>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ardrone_msgs/Mode.h>
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/Imu.h>

class RosThreadIP : public QObject {
	Q_OBJECT

public:
    RosThreadIP();
    virtual ~RosThreadIP();
    bool init();

private:
    ros::NodeHandle * nh;

    // --- ROS sub and pub objects
    ros::Subscriber mode_sub;
    ros::Subscriber navdata_sub;
    ros::Subscriber imu_sub;

    // --- ROS Tf listener objects
    tf::TransformListener tf_pose_listener;

    // --- ROS subscribe functions
    void ModeCallback(const ardrone_msgs::Mode::ConstPtr &msg);
    void NavDataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    // --- Tf listener functions
    void ListenTFTransform();

     // --- Thread object
    QThread * thread;

public slots:
    void run();

signals:
    void receiveCoords(double x, double y, double z, double yaw);
    void receiveMode(QString mode);
    void receiveNavData(float battery, double vx, double vy, double vz);
    void receivedDroneState(uint state);
    void receiveImu(double vyaw);
};

#endif // ROSTHREADIP_H
