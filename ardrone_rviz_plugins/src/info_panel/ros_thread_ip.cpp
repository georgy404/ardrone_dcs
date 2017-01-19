/**
*  This file is part of ardrone_dcs.
*
*  ros_thread_ip is part of info panel rviz plugin.
*  It's provide thread for communicate with ROS system.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include "info_panel/ros_thread_ip.h"


RosThreadIP::RosThreadIP()
{
}

RosThreadIP::~RosThreadIP()
{
    if(ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }

    thread->wait();
}

bool RosThreadIP::init()
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
    mode_sub = nh->subscribe("/ardrone/mode", 10, &RosThreadIP::ModeCallback, this);
    navdata_sub = nh->subscribe("/ardrone/navdata", 10, &RosThreadIP::NavDataCallback, this);
    imu_sub = nh->subscribe("/ardrone/imu", 10, &RosThreadIP::ImuCallback, this);

    thread->start();

    return true;
}


// --- ROS subscribe functions

void RosThreadIP::ModeCallback(const ardrone_msgs::Mode::ConstPtr &msg)
{
    QString modeStr;
    if(msg->mode == ardrone_msgs::Mode::MODE_MANUAL)
        modeStr = "Manual";
    else // (msg->mode == ardrone_msgs::Mode::MODE_AUTO)
        modeStr = "Auto";

    emit receiveMode(modeStr);
}

void RosThreadIP::NavDataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg)
{
    emit receiveNavData(msg->batteryPercent, msg->vx/1000.0, msg->vy/1000.0, msg->vz/1000.0);
    emit receivedDroneState(msg->state);
}

void RosThreadIP::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    emit receiveImu(msg->angular_velocity.z);
}


// --- Tf listener functions

void RosThreadIP::ListenTFTransform()
{
    double x, y, z;
    double roll, pitch, yaw;

    tf::StampedTransform transform;
    try {
        tf_pose_listener.lookupTransform("odom", "base_link", ros::Time(0), transform);

        // get pose
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        z = transform.getOrigin().z();

        // get angles
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(roll, pitch, yaw);
    }
    catch (tf::TransformException ex) {
        std::cerr.flush();
        std::cerr << ex.what() << std::endl;
        ros::Duration(1.0).sleep();
    }

    emit receiveCoords(x, y, z, yaw);
}


// --- ROS thread function

void RosThreadIP::run()
{
    ros::Rate rate(10);

    while (ros::ok()) {
        ListenTFTransform();

        ros::spinOnce();
        rate.sleep();
    }
}
