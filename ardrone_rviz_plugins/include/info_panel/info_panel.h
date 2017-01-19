/**
*  This file is part of ardrone_dcs.
*
*  info_panel is part of info panel rviz plugin.
*  It's provide ardrone diagnostics info.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#pragma once
#ifndef INFOPANEL_H
#define INFOPANEL_H

#include <QtGui>
#include <rviz/panel.h>

#include "info_panel/ros_thread_ip.h"


class InfoPanel: public rviz::Panel
{
    Q_OBJECT

public:
    InfoPanel(QWidget* parent = 0);

    enum InfoNames {
        MODE = 0,
        STATE,
        BATTERY,
        COORDS,
        YAW,
        LINEAR_VEL,
        YAW_VEL
    };

    enum DroneState {
        UNKNOWN    = 0,
        INITED     = 1,
        LANDED     = 2,
        FLYING     = 3,
        HOVERING   = 4,
        TEST       = 5,
        TAKING_OFF = 6,
        FLYING2    = 7,
        LANDING    = 8,
        LOOPING    = 9
    };

private:
    // --- ROS components
    RosThreadIP * ros_thread;

    // --- GUI interface components
    QMap<int, QLabel*> infoLabels;
    QMap<int, QLabel*> infoValues;
    QMap<int, QLabel*> infoQuantities;
    QHBoxLayout * mainLayout;

    // --- GUI interface functions
    inline void CreateWidgets();
    void initLabel(QLabel * label);

public slots:
    void slotReceiveCoords(double x, double y, double z, double yaw);
    void slotReceiveNavData(float battery, double vx, double vy, double vz);
    void slotReceiveImu(double vyaw);
    void slotReceiveMode(QString mode);
    void slotReceivedState(uint state);
};

#endif // INFOPANEL_H
