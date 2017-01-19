/**
*  This file is part of ardrone_dcs.
*
*  control_panel is rviz plugins for provide basec functions.
*  It's create mission, save/load mission, send mission, send control system mode.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#pragma once
#ifndef CONTROLPANEL_H
#define CONTROLPANEL_H

#include <QtGui>
#include <rviz/panel.h>

#include "control_panel/targets_table.h"
#include "control_panel/ros_thread_cp.h"


class ControlPanel: public rviz::Panel
{
    Q_OBJECT

public:
    ControlPanel(QWidget* parent = 0);

private:
    // --- Mode components
    QHBoxLayout * modeHLayout;
    QLabel * modeLabel;
    QComboBox * modeComboBox;
    QPushButton * modeButton;
    int curMode;

    // --- Fly mission table components
    TargetsTable * table;

    QPushButton * addTargetButton;
    QPushButton * delTargetButton;
    QPushButton * clearTargetsButton;
    QPushButton * clearTrajectoryButton;
    QHBoxLayout * ctrMissionTableHLayout;

    QCheckBox * takeoffCheckBox;
    QCheckBox * landCheckBox;
    QHBoxLayout * checkBoxLayout;

    QPushButton * sendMissionButton;
    QPushButton * saveMissionButton;
    QPushButton * loadMissionButton;
    QHBoxLayout * ctrMissionHLayout;

    // --- Main components
    QVBoxLayout * mainVLayout;

    // --- ROS components
    RosThreadCP * ros_thread;

    // --- GUI interface functions
    void CreateWidgets();

    // --- Check data functions
    bool CheckTarget(const ardrone_msgs::NavPose &target);
    bool CheckDroneSate();

public slots:
    void slotSendMode();
    void slotSendMission();
    void slotSaveMission();
    void slotLoadMission();
    void slotAddTarget();
    void slotDeleteTarget();
    void slotClearAllTargets();
    void slotClearTrajectory();
    void slotUpdateMarkerTarget(ardrone_msgs::NavPose target);
    void slotUpdateTableTarget(ardrone_msgs::NavPose target);
};

#endif // CONTROLPANEL_H
