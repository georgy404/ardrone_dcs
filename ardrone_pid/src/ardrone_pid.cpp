/**
*  This file is part of ardrone_dcs.
*
*  Ardron pid node contains pid controller.
*  This node base on tum_ardrone package <https://vision.in.tum.de/data/software/tum_ardrone>.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include "ardrone_pid.h"


ArdronePid::ArdronePid() :
    drone(DroneOdom()),
    target(DroneOdom()),
    b_target_is_set(false),
    last_time_stamp(0.0),
    ros_header_timestamp_base(0.0),
    target_set_at_clock(0.0)
{
    // Create node
    n_h = new ros::NodeHandle;

    // Sub ROS topic
    target_cb = n_h->subscribe("/ardrone/goal", 10, &ArdronePid::TargetCallback, this);
    navdata_cb = n_h->subscribe("/ardrone/navdata", 10, &ArdronePid::ArdroneNavCallback, this);
    imu_cb = n_h->subscribe("/ardrone/imu", 10, &ArdronePid::ArdroneImuCallback, this);
    mode_cb = n_h->subscribe("/ardrone/mode", 10, &ArdronePid::ModeCallback, this);

    // Pub ROS topic
    cmd_vel_pub = n_h->advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
}

ArdronePid::~ArdronePid()
{
}


// -------- Main action functions -----------

void ArdronePid::UpdateControl()
{
    const int rate = 10;
    ros::Rate loop_rate(rate);

    // Set up reading PID controller parameters
    dynamic_reconfigure::Server<ardrone_pid::PidParamsConfig> srv;
    dynamic_reconfigure::Server<ardrone_pid::PidParamsConfig>::CallbackType f;
    f = boost::bind(&ArdronePid::DynConfCallback, this, _1, _2);
    srv.setCallback(f);

    // Main ROS loop
    while (n_h->ok()) {
        // get pose of drone
        ListenTFTransform();

        // pose error (in global frame)
        double new_error[4];
        new_error[0] = target.x - drone.x;
        new_error[1] = target.y - drone.y;
        new_error[2] = target.z - drone.z;
        double target_yaw = atan2( (target.y-drone.y), (target.x-drone.x) );
        new_error[3] = target_yaw - drone.yaw;
        NormalizeAngle(new_error[3]);

        // differential of pose - velocity (in global frame)
        double diff_error[4];
        diff_error[0] = -drone.v_x;
        diff_error[1] = -drone.v_y;
        diff_error[2] = -drone.v_z;
        diff_error[3] = -drone.v_yaw;

        // Check work parameters
        if(mode == ardrone_msgs::Mode::MODE_AUTO) {
            if(b_target_is_set)// && ardrone_state != 2) // if set target and state != Landed
                CalcControl(new_error, diff_error, drone.yaw);
            else
                cmd_vel_pub.publish(geometry_msgs::Twist()); // send zero cmd
        }

        std::copy(new_error, new_error+4, last_err);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ArdronePid::CalcControl(const double * new_error, const double * diff_error, double drone_yaw)
{
    // coeff of scale; maybe it's important
    float agr = agressiveness;

    // --- Calculation of PID controller components

    // P-TERM:
    // p-term: is error (coords)
    double p_term[4] = { new_error[0], new_error[1], new_error[2], new_error[3] };

    // convert p_term (pose) from global farme to local frame
    p_term[0] = cos(drone_yaw) * new_error[0] + sin(drone_yaw) * new_error[1];
    p_term[1] = -sin(drone_yaw) * new_error[0] + cos(drone_yaw) * new_error[1];

    // D-TERM:
    // d-term: just differentiate coords (velocity)
    double d_term[4] = { diff_error[0], diff_error[1], diff_error[2], diff_error[3] };

    // convert d_term (coords) from global farme to local frame
    d_term[0] =  -sin(drone_yaw) * diff_error[0] + cos(drone_yaw) * diff_error[1];
    d_term[1] =  -cos(drone_yaw) * diff_error[0] - sin(drone_yaw) * diff_error[1];

    // I-TERM:
    // integrate & cap
    double sec = GetMS()/1000.0 - last_time_stamp;
    last_time_stamp = GetMS()/1000.0;

    ITermIncrease(i_term[2], new_error[2] * sec, 0.2 / Ki_gaz);
    ITermIncrease(i_term[1], new_error[1] * sec, 0.1 / Ki_rp + (1e-10));
    ITermIncrease(i_term[0], new_error[0] * sec, 0.1 / Ki_rp + (1e-10));

    // kill integral term when first crossing target
    // that is, thargetNew is set, it was set at least 100ms ago, and err changed sign
    for(int i = 0; i < 4 ; i++)
    {
        if( new_target[i] > 0.5 && GetMS()/1000.0 - target_set_at_clock > 0.1 && last_err[i] * new_error[i] < 0 )
        {
            i_term[i] = 0;
            new_target[i] = 0;
        }
    }

    // --- Calculation of control commands

    // Yaw vel
    double ctr_yaw;
    ctr_yaw = Kp_yaw * p_term[3] + Kd_yaw * d_term[3];	// yaw can be translated directly
    ctr_yaw = std::min( max_yaw, std::max(-max_yaw, (double)(ctr_yaw*agr*10)));
    control_cmd.SetYaw(ctr_yaw);

    // X vel
    double ctr_x;
    double cX_p = Kp_rp * p_term[0];
    double cX_d = Kd_rp * d_term[0];
    double cX_i = Ki_rp * i_term[0];
    ctr_x = cX_p + cX_d + cX_i;
    ctr_x = std::min(max_rp,std::max(-max_rp, (double)(ctr_x*agr)));
    control_cmd.SetXVel(ctr_x);

    // Y vel
    double ctr_y;
    double cY_p = Kp_rp * p_term[1];
    double cY_d = Kd_rp * d_term[1];
    double cY_i = Ki_rp * i_term[1];
    ctr_y = cY_p + cY_d + cY_i;
    ctr_y = std::min(max_rp,std::max(-max_rp, (double)(ctr_y*agr)));
    control_cmd.SetYVel(ctr_y);

    // Z vel, gaz
    double ctr_z;
    double gazP = Kp_gaz * p_term[2];
    double gazD = Kd_gaz * d_term[2];
    double gazI = Ki_gaz * i_term[2];
    ctr_z = std::min(max_gaz_rise/rise_fac, std::max(max_gaz_drop, (gazP + gazD + gazI)));
    if(ctr_z > 0)
        ctr_z *= rise_fac;
    control_cmd.SetZVel(ctr_z);

    // --- Publication of control commands

    cmd_vel_pub.publish(control_cmd.getCmd());

    // show info
    std::cout << "---\n";
    std::cout << "pose: " << drone.x << " " << drone.y << " " << drone.z << " " << drone.yaw << "\n";
    std::cout << "target: " << target.x << " " << target.y << " " << target.z << " " << target.yaw << "\n";
    std::cout << "error: " << p_term[0] << " " << p_term[1] << " " << p_term[2] << " " << p_term[3] << "\n";
    std::cout << "ctr_cmd: " << control_cmd.getCmd().linear.x << " " << control_cmd.getCmd().linear.y << " "
                             << control_cmd.getCmd().linear.z << " " << control_cmd.getCmd().angular.z << "\n";
}


// -------- TF transform listener -----------

void ArdronePid::ListenTFTransform()
{
    tf::StampedTransform transform;
    try
    {
        drone_pose_listener.lookupTransform("odom", "base_link", ros::Time(0), transform);

        // get pose
        drone.x = transform.getOrigin().x();
        drone.y = transform.getOrigin().y();
        drone.z = transform.getOrigin().z();

        // get angles
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(drone.roll, drone.pitch, drone.yaw);
    }
    catch (tf::TransformException ex)
    {
        std::cerr.flush();
        std::cerr << ex.what() << std::endl;
        ros::Duration(1.0).sleep();
    }
}


// -------- Subscribe functions -----------

void ArdronePid::TargetCallback(const ardrone_msgs::NavPose::ConstPtr &msg)
{
    b_target_is_set = true;
    target_set_at_clock = GetMS(/*target->header.stamp*/)/1000.0;
    for(int i=0; i<4; i++) {
        new_target[i] = 1.0;
        last_err[i] = i_term[i] = 0.0;
    }

    target.x = msg->x;
    target.y = msg->y;
    target.z = msg->z;
}

void ArdronePid::ArdroneNavCallback(const ardrone_autonomy::Navdata::ConstPtr &navdata)
{
    // get linear velocity and convert mm/s to m/s
    drone.v_x = navdata->vx / 1000.0;
    drone.v_y = navdata->vy / 1000.0;
    drone.v_z = navdata->vz / 1000.0;

    // get ardrone state
    ardrone_state = navdata->state;
}

void ArdronePid::ArdroneImuCallback(const sensor_msgs::Imu::ConstPtr &imu)
{
    drone.v_roll = imu->angular_velocity.x;
    drone.v_pitch = imu->angular_velocity.y;
    drone.v_yaw = imu->angular_velocity.z;
}

void ArdronePid::DynConfCallback(ardrone_pid::PidParamsConfig &config, uint32_t /*level*/)
{
    std::cout << "New PID params\n"; 

    Ki_gaz = config.Ki_gaz;
    Kd_gaz = config.Kd_gaz;
    Kp_gaz = config.Kp_gaz;

    Ki_rp = config.Ki_rp;
    Kd_rp = config.Kd_rp;
    Kp_rp = config.Kp_rp;

    Ki_yaw = config.Ki_yaw;
    Kd_yaw = config.Kd_yaw;
    Kp_yaw = config.Kp_yaw;

    max_gaz_drop = config.max_gaz_drop;
    max_gaz_rise = config.max_gaz_rise;
    max_rp = config.max_rp;
    max_yaw = config.max_yaw;
    agressiveness = config.agressiveness;
    rise_fac = config.rise_fac;
}

void ArdronePid::ModeCallback(const ardrone_msgs::Mode::ConstPtr &msg)
{
    mode = msg->mode;
}


// -------- Other accessory functions -----------

std::ostream & operator << (std::ostream & os, DroneOdom const & odom)
{
    os << "---\n"
       << " pose: "        << odom.x      << " " << odom.y       << " " << odom.z     << "\n"
       << " orientation: " << odom.roll   << " " << odom.pitch   << " " << odom.yaw   << "\n"
       << " linear vel: "  << odom.v_x    << " " << odom.v_y     << " " << odom.v_z   << "\n"
       << " angular vel: " << odom.v_roll << " " << odom.v_pitch << " " << odom.v_yaw << "\n"
       << "\n";

    return os;
}

void ArdronePid::NormalizeAngle(double &angle)
{
    while(angle > M_PI)
        angle -= 2*M_PI;
    while(angle < -M_PI)
        angle += 2*M_PI;
}

double ArdronePid::GetYawFromQuat(const geometry_msgs::Quaternion & q)
{
    tf::Quaternion tf_q;
    tf::quaternionMsgToTF(q, tf_q);
    double target_roll, target_pitch, target_yaw;
    tf::Matrix3x3(tf_q).getRPY(target_roll, target_pitch, target_yaw);
    NormalizeAngle(target_yaw);

    return target_yaw;
}

int ArdronePid::GetMS(ros::Time stamp)
{
    if(ros_header_timestamp_base == 0)
    {
        ros_header_timestamp_base = stamp.sec;
        std::cout << "set ts base to " << ros_header_timestamp_base << std::endl;
    }

    int mss = (stamp.sec - ros_header_timestamp_base) * 1000 + stamp.nsec/1000000;

    if(mss < 0)
        std::cerr << "ERROR: negative timestamp..."<< std::endl;

    return mss;
}

void ArdronePid::ITermIncrease(double & i_term, double new_err, double cap)
{
    if(new_err < 0 && i_term > 0)
        i_term = std::max(0.0, i_term + 2.5 * new_err);
    else if(new_err > 0 && i_term < 0)
        i_term = std::min(0.0, i_term + 2.5 * new_err);
    else
        i_term += new_err;

    if(i_term > cap)
        i_term =  cap;
    if(i_term < -cap)
        i_term =  -cap;
}


