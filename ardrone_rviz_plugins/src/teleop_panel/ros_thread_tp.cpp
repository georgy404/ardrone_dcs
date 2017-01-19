/**
*  This file is part of ardrone_dcs.
*
*  ros_thread_tp is part of teleop panel rviz plugin.
*  It's provide thread for communicate with ROS system.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/


#include "teleop_panel/ros_thread_tp.h"


RosThreadTP::RosThreadTP() :
    isActive(false),
    curLinearVelValue(0.1),
    curAngularVelValue(0.1),
    curVelCmd(geometry_msgs::Twist())
{
}

RosThreadTP::~RosThreadTP()
{
    if(ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }

    thread->wait();
}

bool RosThreadTP::init()
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
    mode_sub = nh->subscribe("/ardrone/mode", 10, &RosThreadTP::ModeCallback, this);

    // Init publisher
    cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    take_off_publisher = nh->advertise<std_msgs::Empty>("/ardrone/takeoff", 10);
    land_publisher = nh->advertise<std_msgs::Empty>("/ardrone/land", 10);
    reset_publisher = nh->advertise<std_msgs::Empty>("/ardrone/reset", 10);

    thread->start();

    return true;
}


// --- Data interface functions

void RosThreadTP::SetCurCmdVel(geometry_msgs::Twist cmdVel)
{
    curVelCmd = cmdVel;
}

void RosThreadTP::ResetCurCmdVel()
{
    curVelCmd = geometry_msgs::Twist();
}

void RosThreadTP::UpLinearVel(float vel)
{
    curLinearVelValue += vel;
    if(curLinearVelValue >= 1.0)
        curLinearVelValue = 1.0;
}

void RosThreadTP::DownLinearVel(float vel)
{
    curLinearVelValue -= vel;
    if(curLinearVelValue <= 0.0)
        curLinearVelValue = 0.0;
}

void RosThreadTP::UpAngularVel(float vel)
{
    curAngularVelValue += vel;
    if(curAngularVelValue >= 1.0)
        curAngularVelValue = 1.0;
}

void RosThreadTP::DownAngularVel(float vel)
{
    curAngularVelValue -= vel;
    if(curAngularVelValue <= 0.0)
        curAngularVelValue = 0.0;
}

float RosThreadTP::GetLinearVel()
{
    return curLinearVelValue;
}

float RosThreadTP::GetAngularVel()
{
    return curAngularVelValue;
}


// --- Data send functions

void RosThreadTP::SendTakeOffCmd()
{
    if(isActive)
        take_off_publisher.publish(std_msgs::Empty());
}

void RosThreadTP::SendLandCmd()
{
    if(isActive)
        land_publisher.publish(std_msgs::Empty());
}

void RosThreadTP::SendResetCmd()
{
    if(isActive)
        reset_publisher.publish(std_msgs::Empty());
}

void RosThreadTP::SendCmdVel()
{
    if(isActive)
        cmd_vel_pub.publish(curVelCmd);
}


// --- ROS subscribe functions

void RosThreadTP::ModeCallback(const ardrone_msgs::Mode::ConstPtr &msg)
{
    if(msg->mode == ardrone_msgs::Mode::MODE_MANUAL)
        isActive = true;
    else // (msg->mode == ardrone_msgs::Mode::MODE_AUTO)
        isActive = false;

    emit working(isActive);
}


// --- ROS thread function

void RosThreadTP::run()
{
    ros::Rate rate(10);

    while (ros::ok()) {
        SendCmdVel();

        ros::spinOnce();
        rate.sleep();
    }
}
