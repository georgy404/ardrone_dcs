/**
*  This file is part of ardrone_dcs.
*
*  info_panel is part of info panel rviz plugin.
*  It's provide ardrone diagnostics info.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include <pluginlib/class_list_macros.h>
#include "info_panel/info_panel.h"

InfoPanel::InfoPanel(QWidget* parent):
    rviz::Panel(parent)
{
    // ROS init
    ros_thread = new RosThreadIP();
    ros_thread->init();

    // GUI init
    CreateWidgets();

    // Connect signals and slots
    connect(ros_thread, SIGNAL(receiveCoords(double,double,double,double)),
                            this, SLOT(slotReceiveCoords(double,double,double,double)));
    connect(ros_thread, SIGNAL(receiveNavData(float,double,double,double)),
                            this, SLOT(slotReceiveNavData(float,double,double,double)));
    connect(ros_thread, SIGNAL(receiveImu(double)),       this, SLOT(slotReceiveImu(double)));
    connect(ros_thread, SIGNAL(receiveMode(QString)),     this, SLOT(slotReceiveMode(QString)));
    connect(ros_thread, SIGNAL(receivedDroneState(uint)), this, SLOT(slotReceivedState(uint)));
}


// --- GUI interface functions

void InfoPanel::CreateWidgets()
{
    // Create info labels
    infoLabels.insert(MODE,       new QLabel(tr("Mode")));
    infoLabels.insert(STATE,      new QLabel(tr("State")));
    infoLabels.insert(BATTERY,    new QLabel(tr("Battery")));
    infoLabels.insert(COORDS,     new QLabel(tr("Coords")));
    infoLabels.insert(YAW,        new QLabel(tr("Yaw")));
    infoLabels.insert(LINEAR_VEL, new QLabel(tr("Linear vel")));
    infoLabels.insert(YAW_VEL,    new QLabel(tr("Yaw vel")));

    // Create info quantities
    infoQuantities.insert(MODE,       new QLabel(tr(":")));
    infoQuantities.insert(STATE,      new QLabel(tr(":")));
    infoQuantities.insert(BATTERY,    new QLabel(tr("[%]:")));
    infoQuantities.insert(COORDS,     new QLabel(tr("[m]:")));
    infoQuantities.insert(YAW,        new QLabel(tr("[deg]:")));
    infoQuantities.insert(LINEAR_VEL, new QLabel(tr("[m/s]:")));
    infoQuantities.insert(YAW_VEL,    new QLabel(tr("rad/s:")));

    // Create layouts
    QVBoxLayout * labelsLayout = new QVBoxLayout;
    QVBoxLayout * valueLayout = new QVBoxLayout;
    QVBoxLayout * quantitiesLayout = new QVBoxLayout;

    // Create value labels and init all labels
    QMap<int, QLabel*>::iterator i;
    for(i = infoLabels.begin(); i != infoLabels.end(); ++i) {
        initLabel(i.value());

        initLabel(infoQuantities.value(i.key()));

        infoValues.insert(i.key(), new QLabel(" --- "));
        initLabel(infoValues.value(i.key()));

        labelsLayout->addWidget(i.value(),                         0, Qt::AlignLeft);
        quantitiesLayout->addWidget(infoQuantities.value(i.key()), 0, Qt::AlignRight);
        valueLayout->addWidget(infoValues.value(i.key()),          0, Qt::AlignLeft);
    }

    // Create main layout and add other layout
    mainLayout = new QHBoxLayout;
    mainLayout->addLayout(labelsLayout);
    mainLayout->addLayout(quantitiesLayout);
    mainLayout->addLayout(valueLayout);
    mainLayout->addStretch(1000);

    this->setLayout(mainLayout);
}

void InfoPanel::initLabel(QLabel *label)
{
    QFont font = this->font();
    font.setPointSize(10);
    label->setFont(font);
}


// --- Slots

void InfoPanel::slotReceiveCoords(double x, double y, double z, double yaw)
{
    infoValues.value(COORDS)->setText(QString::number(round(x*1000)/1000.) + QString("; ") +
                                      QString::number(round(y*1000)/1000.) + QString("; ") +
                                      QString::number(round(z*1000)/1000.));
    infoValues.value(YAW)->setText(QString::number(round((yaw*180.0/M_PI)*1000)/1000.));
}

void InfoPanel::slotReceiveNavData(float battery, double vx, double vy, double vz)
{
    infoValues.value(BATTERY)->setText(QString::number(battery));
    infoValues.value(LINEAR_VEL)->setText(QString::number(round(vx*1000)/1000.) + QString("; ") +
                                          QString::number(round(vy*1000)/1000.) + QString("; ") +
                                          QString::number(round(vz*1000)/1000.));
}

void InfoPanel::slotReceiveImu(double vyaw)
{
    infoValues.value(YAW_VEL)->setText(QString::number(round(vyaw*1000)/1000.));
}

void InfoPanel::slotReceiveMode(QString mode)
{
    infoValues.value(MODE)->setText(mode);
}

void InfoPanel::slotReceivedState(uint state)
{
    QString stateStr;

    switch (state) {
    case UNKNOWN:
        stateStr = "Unknown";
        break;
    case INITED:
        stateStr = "Inited";
        break;
    case LANDED:
        stateStr = "Landed";
        break;
    case FLYING:
        stateStr = "Flying";
        break;
    case HOVERING:
        stateStr = "Hovering";
        break;
    case TEST:
        stateStr = "Test";
        break;
    case TAKING_OFF:
        stateStr = "Taking off";
        break;
    case FLYING2:
        stateStr = "Flying";
        break;
    case LANDING:
        stateStr = "Landing";
        break;
    case LOOPING:
        stateStr = "Looping";
        break;
    default:
        break;
    }

    infoValues.value(STATE)->setText(stateStr);
}


PLUGINLIB_EXPORT_CLASS(InfoPanel, rviz::Panel)
