/**
*  This file is part of ardrone_dcs.
*
*  teleop_panel is part of teleop panel rviz plugin.
*  It's provide GUI interface for manual control ardrone.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include <pluginlib/class_list_macros.h>
#include "teleop_panel/teleop_panel.h"

TeleopPanel::TeleopPanel(QWidget* parent):
    rviz::Panel(parent),
    isModeActive(false),
    isFocusActive(false)
{
    // ROS init
    ros_thread = new RosThreadTP();
    ros_thread->init();

    // GUI init
    CreateWidgets();
    this->setFocusPolicy(Qt::StrongFocus);

    // Connect signals and slots
    connect(ros_thread, SIGNAL(working(bool)), this, SLOT(slotSetIsActive(bool)));
}


// --- GUI interface functions

void TeleopPanel::CreateWidgets()
{
    // Ctr buttons header label
    ctrButtonsLabel = new QLabel(tr("Control keys"));

    // Init ctr buttons maps
    CreateCtrButtons();

    // Init ctr velocity sliders
    angularVelSliderLabel = new QLabel(tr("Angular vel"));
    angularVelSlider = CreateVelSlider();
    connect(angularVelSlider, SIGNAL(valueChanged(int)), this, SLOT(slotAngularVelChanged(int)));

    angularVelLine = CreateVelLine();
    angularVelLine->setText(QString::number(ros_thread->GetAngularVel()));
    angularVelLine->setToolTip("Angular velocity value");

    linearVelSliderLabel = new QLabel(tr("Linear vel"));
    linearVelSlider = CreateVelSlider();
    connect(linearVelSlider, SIGNAL(valueChanged(int)), this, SLOT(slotLinearVelChanged(int)));

    linearVelLine = CreateVelLine();
    linearVelLine->setText(QString::number(ros_thread->GetLinearVel()));
    linearVelLine->setToolTip("Linear velocity value");

    // Init focus status label
    focusActiveLabel = new QLabel();
    SetFocusActivePanel(false);

    // Create layouts and add widgets to layouts
    CreateLayouts();
}

void TeleopPanel::CreateCtrButtons()
{
    // Create ctr buttons widgets
    ctrButtons.insert(MOVE_UP,       new QPushButton(tr("W")));
    ctrButtons.insert(MOVE_DOWN,     new QPushButton(tr("S")));
    ctrButtons.insert(TURN_LEFT,     new QPushButton(tr("A")));
    ctrButtons.insert(TURN_RIGHT,    new QPushButton(tr("D")));

    ctrButtons.insert(RESET,         new QPushButton(tr("R")));
    ctrButtons.insert(TAKE_OFF,      new QPushButton(tr("T")));
    ctrButtons.insert(LAND,          new QPushButton(tr("G")));

    ctrButtons.insert(MOVE_FORWARD,  new QPushButton(tr("I")));
    ctrButtons.insert(MOVE_BACKWARD, new QPushButton(tr("K")));
    ctrButtons.insert(MOVE_LEFT,     new QPushButton(tr("J")));
    ctrButtons.insert(MOVE_RIGHT,    new QPushButton(tr("L")));

    // Set tool tip to ctr buttons
    ctrButtons.value(MOVE_UP)->setToolTip("Move up");
    ctrButtons.value(MOVE_DOWN)->setToolTip("Move down");
    ctrButtons.value(TURN_LEFT)->setToolTip("Turn left");
    ctrButtons.value(TURN_RIGHT)->setToolTip("Turn right");

    ctrButtons.value(RESET)->setToolTip("Reset");
    ctrButtons.value(TAKE_OFF)->setToolTip("Take off");
    ctrButtons.value(LAND)->setToolTip("Land");

    ctrButtons.value(MOVE_FORWARD)->setToolTip("Move forward");
    ctrButtons.value(MOVE_BACKWARD)->setToolTip("Move backward");
    ctrButtons.value(MOVE_LEFT)->setToolTip("Move left");
    ctrButtons.value(MOVE_RIGHT)->setToolTip("Move right");

    QMap<int, QPushButton*>::iterator i;
    for(i = ctrButtons.begin(); i != ctrButtons.end(); ++i) {
        i.value()->setObjectName(QString::number(i.key()));
        i.value()->setMaximumSize(30, 30);
        i.value()->setFocusPolicy(Qt::NoFocus);
        i.value()->setEnabled(false);

        connect(i.value(), SIGNAL(pressed()), this, SLOT(slotKeyWidgetPressed()));
        connect(i.value(), SIGNAL(released()), this, SLOT(slotKeyWidgetReleased()));
    }
}

QSlider * TeleopPanel::CreateVelSlider()
{
    QSlider * slider = new QSlider();
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(0, 10);
    slider->setSingleStep(1);
    slider->setValue(1);
    slider->setEnabled(false);
    slider->setFocusPolicy(Qt::NoFocus);

    return slider;
}

QLineEdit * TeleopPanel::CreateVelLine()
{
    QLineEdit * line = new QLineEdit();
    line->setReadOnly(true);
    line->setFocusPolicy(Qt::NoFocus);
    line->setMaximumSize(40, 30);

    QFont font = this->font();
    font.setPointSize(9);
    line->setFont(font);

    return line;
}

void TeleopPanel::CreateLayouts()
{
    // Left ctr buttons
    QVBoxLayout * leftKeyLayout1 = new QVBoxLayout;
    leftKeyLayout1->addStretch(1000);
    leftKeyLayout1->addWidget(ctrButtons.value(TURN_LEFT));

    QVBoxLayout * leftKeyLayout2 = new QVBoxLayout;
    leftKeyLayout2->addStretch(1000);
    leftKeyLayout2->addWidget(ctrButtons.value(MOVE_UP));
    leftKeyLayout2->addWidget(ctrButtons.value(MOVE_DOWN));

    QVBoxLayout * leftKeyLayout3 = new QVBoxLayout;
    leftKeyLayout3->addStretch(1000);
    leftKeyLayout3->addWidget(ctrButtons.value(TURN_RIGHT));

    QHBoxLayout * leftKeysLayout = new QHBoxLayout;
    leftKeysLayout->addLayout(leftKeyLayout1);
    leftKeysLayout->addLayout(leftKeyLayout2);
    leftKeysLayout->addLayout(leftKeyLayout3);

    // Angular velocity slider
    QHBoxLayout * angularSliderLayout = new QHBoxLayout;
    angularSliderLayout->addWidget(angularVelSlider);
    angularSliderLayout->addWidget(angularVelLine);

    // Left ctr buttons and angular velocity slider
    QVBoxLayout * leftCtrLayout = new QVBoxLayout;
    leftCtrLayout->addLayout(leftKeysLayout);
    leftCtrLayout->addWidget(angularVelSliderLabel, 0, Qt::AlignCenter);
    leftCtrLayout->addLayout(angularSliderLayout);

    // Center ctr buttons
    QVBoxLayout * centerKeyLayout = new QVBoxLayout;
    centerKeyLayout->addStretch(1000);
    centerKeyLayout->addWidget(ctrButtons.value(RESET));
    centerKeyLayout->addWidget(ctrButtons.value(TAKE_OFF));
    centerKeyLayout->addWidget(ctrButtons.value(LAND));
    centerKeyLayout->addStretch(2000);

    // Right ctr buttons
    QVBoxLayout * rightKeyLayout1 = new QVBoxLayout;
    rightKeyLayout1->addStretch(1000);
    rightKeyLayout1->addWidget(ctrButtons.value(MOVE_LEFT));

    QVBoxLayout * rightKeyLayout2 = new QVBoxLayout;
    rightKeyLayout2->addStretch(1000);
    rightKeyLayout2->addWidget(ctrButtons.value(MOVE_FORWARD));
    rightKeyLayout2->addWidget(ctrButtons.value(MOVE_BACKWARD));

    QVBoxLayout * rightKeyLayout3 = new QVBoxLayout;
    rightKeyLayout3->addStretch(1000);
    rightKeyLayout3->addWidget(ctrButtons.value(MOVE_RIGHT));

    QHBoxLayout * rightKeysLayout = new QHBoxLayout;
    rightKeysLayout->addLayout(rightKeyLayout1);
    rightKeysLayout->addLayout(rightKeyLayout2);
    rightKeysLayout->addLayout(rightKeyLayout3);

    // Linear velocity slider
    QHBoxLayout * linearSliderLayout = new QHBoxLayout;
    linearSliderLayout->addWidget(linearVelSlider);
    linearSliderLayout->addWidget(linearVelLine);

    // Right ctr buttons and linear velocity slider
    QVBoxLayout * rightCtrLayout = new QVBoxLayout;
    rightCtrLayout->addLayout(rightKeysLayout);
    rightCtrLayout->addWidget(linearVelSliderLabel, 0, Qt::AlignCenter);
    rightCtrLayout->addLayout(linearSliderLayout);

    // Ctr keys layout
    QHBoxLayout * ctrKeysLayout = new QHBoxLayout;
    ctrKeysLayout->addLayout(leftCtrLayout);
    ctrKeysLayout->addLayout(centerKeyLayout);
    ctrKeysLayout->addLayout(rightCtrLayout);

    // Main layout
    mainLayout = new QVBoxLayout;
    mainLayout->addWidget(ctrButtonsLabel,  0, Qt::AlignHCenter);
    mainLayout->addLayout(ctrKeysLayout);
    mainLayout->addWidget(focusActiveLabel, 0, Qt::AlignHCenter);

    this->setLayout(mainLayout);
}


// --- GUI control functions

void TeleopPanel::EnableWidgets(bool enable)
{
    linearVelSlider->setEnabled(enable);
    angularVelSlider->setEnabled(enable);

    QMap<int, QPushButton*>::iterator i;
    for(i = ctrButtons.begin(); i != ctrButtons.end(); ++i) {
        i.value()->setEnabled(enable);
    }
}

void TeleopPanel::SetFocusActivePanel(bool focus)
{
    QPalette palette;
    QString text;
    QString toolTripText;
    if(focus) {
        palette.setColor(focusActiveLabel->foregroundRole(), Qt::darkGreen);
        text = "The panel is active";
        toolTripText = "Use keyboard keys or widgets for sending commands";
    }
    else { // if(!focus)
        palette.setColor(focusActiveLabel->foregroundRole(), Qt::darkRed);
        text = "The panel is not active";
        toolTripText = "Choose manual mode and focus on it for activate";
    }

    focusActiveLabel->setPalette(palette);
    focusActiveLabel->setText(text);
    focusActiveLabel->setToolTip(toolTripText);

    EnableWidgets(focus);
}


// --- Ctr keys functions

void TeleopPanel::SendControl(int ctrKey)
{
    geometry_msgs::Twist cmdVel;

    switch (ctrKey)
    {
    case MOVE_UP:
        cmdVel.linear.z = ros_thread->GetLinearVel();
        break;

    case MOVE_DOWN:
        cmdVel.linear.z = -ros_thread->GetLinearVel();
        break;

    case TURN_LEFT:
        cmdVel.angular.z = ros_thread->GetAngularVel();
        break;

    case TURN_RIGHT:
        cmdVel.angular.z = -ros_thread->GetAngularVel();
        break;

    case MOVE_FORWARD:
        cmdVel.linear.x = ros_thread->GetLinearVel();
        break;

    case MOVE_BACKWARD:
        cmdVel.linear.x = -ros_thread->GetLinearVel();
        break;

    case MOVE_LEFT:
        cmdVel.linear.y = ros_thread->GetLinearVel();
        break;

    case MOVE_RIGHT:
        cmdVel.linear.y = -ros_thread->GetLinearVel();
        break;

    case TAKE_OFF:
        ros_thread->SendTakeOffCmd();
        break;

    case LAND:
        ros_thread->SendLandCmd();
        break;

    case RESET:
        ros_thread->SendResetCmd();
        break;

    case MOVE_VEL_UP:
        ros_thread->UpLinearVel();
        linearVelSlider->setValue(ros_thread->GetLinearVel()*10);
        return;

    case MOVE_VEL_DOWN:
        ros_thread->DownLinearVel();
        linearVelSlider->setValue(ros_thread->GetLinearVel()*10);
        return;

    case TURN_VEL_UP:
        ros_thread->UpAngularVel();
        angularVelSlider->setValue(ros_thread->GetAngularVel()*10);
        return;

    case TURN_VEL_DOWN:
        ros_thread->DownAngularVel();
        angularVelSlider->setValue(ros_thread->GetAngularVel()*10);
        return;

    default:
        return;
    }

    ros_thread->SetCurCmdVel(cmdVel);
    ctrButtons.value(ctrKey)->setDown(true);
}

void TeleopPanel::ResetControl(int ctrKey)
{
    ros_thread->ResetCurCmdVel();

    if(ctrButtons.contains(ctrKey))
        ctrButtons.value(ctrKey)->setDown(false);
}


// --- Keyboard event functions

void TeleopPanel::keyPressEvent(QKeyEvent* event)
{
    if(!isModeActive || !isFocusActive)
        return;

    rviz::Panel::keyPressEvent(event);

    SendControl(event->key());
}

void TeleopPanel::keyReleaseEvent(QKeyEvent* event)
{
    if(!isModeActive || !isFocusActive)
        return;

    if(event->isAutoRepeat())
        return;

    ResetControl(event->key());
}


// --- Focus event functions

void TeleopPanel::focusInEvent(QFocusEvent * /*event*/)
{
    isFocusActive = true;
    if(!isModeActive)
        return;

    SetFocusActivePanel(isFocusActive);
}

void TeleopPanel::focusOutEvent(QFocusEvent * /*event*/)
{
    isFocusActive = false;
    if(!isModeActive)
        return;

    SetFocusActivePanel(isFocusActive);
}


// --- Slots

void TeleopPanel::slotLinearVelChanged(int vel)
{
    if(!isModeActive || !isFocusActive)
        return;

    float curVel = ros_thread->GetLinearVel();
    float diffVel = (float)vel/10. - curVel;

    if(diffVel > 0)
        ros_thread->UpLinearVel(diffVel);
    else // if(diffVel < 0)
        ros_thread->DownLinearVel(-diffVel);

    linearVelLine->setText(QString::number(ros_thread->GetLinearVel()));
}

void TeleopPanel::slotAngularVelChanged(int vel)
{
    if(!isModeActive || !isFocusActive)
        return;

    float curVel = ros_thread->GetAngularVel();
    float diffVel = (float)vel/10. - curVel;

    if(diffVel > 0)
        ros_thread->UpAngularVel(diffVel);
    else // if(diffVel < 0)
        ros_thread->DownAngularVel(-diffVel);

    angularVelLine->setText(QString::number(ros_thread->GetAngularVel()));
}

void TeleopPanel::slotSetIsActive(bool work)
{
    isModeActive = work;

    if(!isFocusActive)
        return;

    EnableWidgets(isModeActive);
}

void TeleopPanel::slotKeyWidgetPressed()
{
    int key = sender()->objectName().toInt();
    SendControl(key);
}

void TeleopPanel::slotKeyWidgetReleased()
{
    int key = sender()->objectName().toInt();
    ResetControl(key);
}


PLUGINLIB_EXPORT_CLASS(TeleopPanel, rviz::Panel)
