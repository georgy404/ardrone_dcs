/**
*  This file is part of ardrone_dcs.
*
*  teleop_panel is part of teleop panel rviz plugin.
*  It's provide GUI interface for manual control ardrone.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#pragma once
#ifndef TELEOPPANEL_H
#define TELEOPPANEL_H

#include <QtGui>
#include <rviz/panel.h>

#include "geometry_msgs/Twist.h"

#include "teleop_panel/ros_thread_tp.h"


class TeleopPanel: public rviz::Panel
{
    Q_OBJECT

public:
    TeleopPanel(QWidget* parent = 0);

    // --- Control keys codes
    enum CtrKeys {
        MOVE_UP       = Qt::Key_W,
        MOVE_DOWN     = Qt::Key_S,
        MOVE_FORWARD  = Qt::Key_I,
        MOVE_BACKWARD = Qt::Key_K,
        MOVE_LEFT     = Qt::Key_J,
        MOVE_RIGHT    = Qt::Key_L,
        TURN_LEFT     = Qt::Key_A,
        TURN_RIGHT    = Qt::Key_D,
        RESET         = Qt::Key_R,
        TAKE_OFF      = Qt::Key_T,
        LAND          = Qt::Key_G,
        MOVE_VEL_UP   = Qt::Key_Up,
        MOVE_VEL_DOWN = Qt::Key_Down,
        TURN_VEL_UP   = Qt::Key_Right,
        TURN_VEL_DOWN = Qt::Key_Left,
    };

private:
    // --- ROS components
    RosThreadTP * ros_thread;

    // --- GUI interface components
    QLabel * ctrButtonsLabel;
    QMap <int, QPushButton*> ctrButtons;
    QLabel * linearVelSliderLabel;
    QSlider * linearVelSlider;
    QLineEdit * linearVelLine;
    QLabel * angularVelSliderLabel;
    QSlider * angularVelSlider;
    QLineEdit * angularVelLine;
    QLabel * focusActiveLabel;
    QVBoxLayout * mainLayout;

    // --- GUI interface functions
    inline void CreateWidgets();
    inline void CreateCtrButtons();
    inline void CreateLayouts();
    QSlider * CreateVelSlider();
    QLineEdit * CreateVelLine();

    // --- GUI control functions
    void SetFocusActivePanel(bool focus);
    void EnableWidgets(bool enable);

    // --- Ctr keys functions
    void SendControl(int ctrKey);
    void ResetControl(int ctrKey);

    // --- Check panel active variables
    bool isModeActive;
    bool isFocusActive;

protected:
    void keyPressEvent(QKeyEvent * event);
    void keyReleaseEvent(QKeyEvent * event);
    void focusInEvent(QFocusEvent * event);
    void focusOutEvent(QFocusEvent * event);

public slots:
    void slotSetIsActive(bool work);

private slots:
    void slotLinearVelChanged(int vel);
    void slotAngularVelChanged(int vel);
    void slotKeyWidgetPressed();
    void slotKeyWidgetReleased();
};

#endif // TELEOPPANEL_H
