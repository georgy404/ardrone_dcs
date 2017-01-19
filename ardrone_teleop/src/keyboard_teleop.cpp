/**
*  This file is part of ardrone_dcs.
*
*  keyboard_teleop is node for streaming commands (velocity) to ardrone using keyboard.
*  It's using for draw trajectory and connection line of waypoints.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include <stdio.h>
#include <curses.h>
#include <termios.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"

#define MOVE_FORWARD   'i'
#define MOVE_BACKWARD  'k'
#define MOVE_LEFT      'j'
#define MOVE_RIGHT     'l'
#define MOVE_UP        'w'
#define MOVE_DOWN      's'
#define TURN_RIGHT     'd'
#define TUR_LEFT       'a'
#define TAKE_OFF       't'
#define LAND           'g'
#define RESET          'b'
#define TURN_VEL_UP    ']'
#define TURN_VEL_DOWN  '['
#define MOVE_VEL_UP    '='
#define MOVE_VEL_DOWN  '-'

static struct termios stored;

ros::Publisher vel_publisher;
ros::Publisher land_publisher;
ros::Publisher take_off_publisher;
ros::Publisher reset_publisher;

geometry_msgs::Twist cur_vel;
float vel_move_value = 0.1;
float vel_turn_value = 0.1;

inline void help_singboard()
{
    int i = 2; // second terminal line
    printf( "%c[%d;%dH", 27, i++, 1 );
    printf( "=====================================================================\n");
    printf( "%c[%d;%dH", 27, i++, 1 );
    printf( " You are going to take over control of the drone! Ha-ha, good luck!!!\n");
    printf( "%c[%d;%dH", 27, i++, 1 );
    printf( " Keys for control:\n");
    printf( "%c[%d;%dH", 27, i++, 1 );
    printf( "   t/g - takeoff/land\n");
    printf( "%c[%d;%dH", 27, i++, 1 );
    printf( "   -/= - down/up move velocity\n");
    printf( "%c[%d;%dH", 27, i++, 1 );
    printf( "   [/] - down/up turn velocity\n");
    printf( "%c[%d;%dH", 27, i++, 1 );
    printf( "   w/s - move up/down\n");
    printf( "%c[%d;%dH", 27, i++, 1 );
    printf( "   a/d - turn left/right\n");
    printf( "%c[%d;%dH", 27, i++, 1 );
    printf( "   i/k - move forward/backward\n");
    printf( "%c[%d;%dH", 27, i++, 1 );
    printf( "   j/l - move left/right\n");
    printf( "%c[%d;%dH", 27, i++, 1 );
    printf( "   other key for STOP\n\n");
}

void print_vel()
{
    std::cout <<    "move vel = " << vel_move_value*100
              << ";  turn vel = " << vel_turn_value*100;
}

inline void init_terminal()
{
    struct termios settings;

    tcgetattr(0, &stored);

    settings = stored;

    settings.c_lflag &= (~ICANON);
    settings.c_lflag &= (~ECHO);
    settings.c_cc[VTIME] = 0;
    settings.c_cc[VMIN] = 1;

    tcsetattr(0, TCSANOW, &settings);
}

void vel_up(float &vel)
{
    vel += 0.1;
    if(vel >= 1.0)
        vel = 1.0;
}

void vel_down(float &vel)
{
    vel -= 0.1;
    if(vel <= 0.0)
        vel = 0.0;
}

void send_control(char input_ch)
{
    switch (input_ch) {
    case MOVE_FORWARD:
        std::cout << "move forward" << std::endl;
        cur_vel.linear.x = vel_move_value;
        vel_publisher.publish(cur_vel);
        break;

    case MOVE_BACKWARD:
        std::cout << "move backward" << std::endl;
        cur_vel.linear.x = -vel_move_value;
        vel_publisher.publish(cur_vel);
        break;

    case MOVE_LEFT:
        std::cout << "move left" << std::endl;
        cur_vel.linear.y = vel_move_value;
        vel_publisher.publish(cur_vel);
        break;

    case MOVE_RIGHT:
        std::cout << "move right" << std::endl;
        cur_vel.linear.y = -vel_move_value;
        vel_publisher.publish(cur_vel);
        break;

    case MOVE_UP:
        std::cout << "move up" << std::endl;
        cur_vel.linear.z = vel_move_value;
        vel_publisher.publish(cur_vel);
        break;

    case MOVE_DOWN:
        std::cout << "move down" << std::endl;
        cur_vel.linear.z = -vel_move_value;
        vel_publisher.publish(cur_vel);
        break;

    case TURN_RIGHT:
        std::cout << "turn right" << std::endl;
        cur_vel.angular.z = -vel_turn_value;
        vel_publisher.publish(cur_vel);
        break;

    case TUR_LEFT:
        std::cout << "turn left" << std::endl;
        cur_vel.angular.z = vel_turn_value;
        vel_publisher.publish(cur_vel);
        break;

    case TAKE_OFF:
        std::cout << "take off" << std::endl;
        take_off_publisher.publish(std_msgs::Empty());
        break;

    case LAND:
        std::cout << "land" << std::endl;
        land_publisher.publish(std_msgs::Empty());
        break;
        
    case RESET:
        std::cout << "reset" << std::endl;
        reset_publisher.publish(std_msgs::Empty());
        break;

    case MOVE_VEL_UP:
        std::cout << "move vel up" << std::endl;
        vel_up(vel_move_value);
        break;

    case MOVE_VEL_DOWN:
        std::cout << "move vel down" << std::endl;
        vel_down(vel_move_value);
        break;

    case TURN_VEL_UP:
        std::cout << "turn vel up" << std::endl;
        vel_up(vel_turn_value);
        break;

    case TURN_VEL_DOWN:
        std::cout << "turn vel down" << std::endl;
        vel_down(vel_turn_value);
        break;

    default:
        std::cout << "stop" << std::endl;
        cur_vel = geometry_msgs::Twist();
        vel_publisher.publish(cur_vel);
        break;
    }
}

int main(int argc, char **argv)
{
    // --- init ROS
    ros::init(argc, argv, "ardrone_keyboard_teleop");
    ros::NodeHandle nh;
    vel_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    take_off_publisher = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
    land_publisher = nh.advertise<std_msgs::Empty>("/ardrone/land", 1000);
    reset_publisher = nh.advertise<std_msgs::Empty>("/ardrone/reset", 1000);

    // --- init terminals
    init_terminal();

    // --- main loop
    while(ros::ok()) {
        initscr();

        timeout(300);
        send_control(getch());

        // print info and decor msgs to stdin
        help_singboard();
        printf("%c[2J", 27); // clear terminal
        printf("%c[%d;%dH", 27, 14, 3); // set cursor to (14,3)
        print_vel();
        printf("%c[%d;%dH >>>  ", 27, 16, 3); // set cursor to (16,3)

        ros::spinOnce();
    }

    endwin();
    tcsetattr(0, TCSANOW, &stored);

    return 0;
}
