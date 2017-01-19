/**
*  This file is part of ardrone_dcs.
*
*  Ardron pid node contains pid controller.
*  This node base on tum_ardrone package <https://vision.in.tum.de/data/software/tum_ardrone>.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#pragma once
#ifndef __ARDRONE_PID_H
#define __ARDRONE_PID_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_listener.h"
#include "dynamic_reconfigure/server.h"
#include "ardrone_pid/PidParamsConfig.h"
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_msgs/Mode.h"
#include "ardrone_msgs/NavPose.h"


class DroneOdom
{
    // Pose + angle (Euler) + velocity;
    // using axes like in ROS
public:
    DroneOdom() :
        x(0.0), v_x(0.0), roll(0.0),  v_roll(0.0),
        y(0.0), v_y(0.0), pitch(0.0), v_pitch(0.0),
        z(0.0), v_z(0.0), yaw(0.0),   v_yaw(0.0)
    {
    }

    // pose [m]
    double x;  // forward
    double y;  // left
    double z;  // up

    // linear velocity [m/s]
    double v_x;
    double v_y;
    double v_z;

    // angle [rad]
    double roll;  // x-axes
    double pitch; // y-axes
    double yaw;   // z-axes

    // angular velocity [rad/s]
    double v_roll;
    double v_pitch;
    double v_yaw;

    friend std::ostream & operator << (std::ostream & os, const DroneOdom & odom);
};


class ControlCmd
{
    // Interface for controll commands
private:
    geometry_msgs::Twist cmd;

public:
    // get cmd
    const geometry_msgs::Twist & getCmd() { return cmd; }

    // set parametrs
    void SetXVel(const double & vel) { cmd.linear.x = vel; }
    void SetYVel(const double & vel) { cmd.linear.y = vel; }
    void SetZVel(const double & vel) { cmd.linear.z = vel; }
    void SetYaw(double yaw) { cmd.angular.z = yaw; }
};


class ArdronePid
{
public:
    ArdronePid();
    ~ArdronePid();

    // main loop
    void UpdateControl();

    // nodehandle for ROS magic
    ros::NodeHandle * n_h;

private:
    // current velocity command
    ControlCmd control_cmd;

    // drone - where i now
    DroneOdom drone;

    // target - where i want going
    DroneOdom target;
    bool b_target_is_set;

    // time for integrate
    double last_time_stamp;
    unsigned int ros_header_timestamp_base;
    double target_set_at_clock;

    // variables for integrate term of pid controller
    double last_err[4];
    double i_term[4];
    double new_target[4];

    // Control system mode
    int mode;
    int ardrone_state;

    // ROS objects for communication
    tf::TransformListener drone_pose_listener;
    ros::Subscriber navdata_cb;
    ros::Subscriber imu_cb;
    ros::Subscriber mode_cb;
    ros::Subscriber target_cb;
    ros::Publisher cmd_vel_pub;

    // PID control parameters; settable via dynamic_reconfigure
    double max_yaw;
    double max_rp;
    double max_gaz_rise;
    double max_gaz_drop;

    double rise_fac;
    double agressiveness;

    double Ki_yaw;
    double Kd_yaw;
    double Kp_yaw;

    double Ki_gaz;
    double Kd_gaz;
    double Kp_gaz;

    double Ki_rp;
    double Kd_rp;
    double Kp_rp;

    // other accessory functions
    void NormalizeAngle(double & angle);
    double GetYawFromQuat(const geometry_msgs::Quaternion & q);
    void ITermIncrease(double & i_term, double new_err, double cap);
    int GetMS(ros::Time stamp = ros::Time::now());

    // tf transform function
    void ListenTFTransform();

    // callback functions
    void ModeCallback(const ardrone_msgs::Mode::ConstPtr &msg);
    void TargetCallback(const ardrone_msgs::NavPose::ConstPtr &msg);
    void ArdroneNavCallback(const ardrone_autonomy::Navdata::ConstPtr &navdata);
    void ArdroneImuCallback(const sensor_msgs::Imu::ConstPtr &imu);
    void DynConfCallback(ardrone_pid::PidParamsConfig &config, uint32_t level);

    // pid regulator calculation actions
    void CalcControl(const double * new_error, const double * diff_error, double drone_yaw);
};


#endif /* __ARDRONE_PID_H */
