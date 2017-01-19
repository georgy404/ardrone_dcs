/**
*  This file is part of ardrone_dcs.
*
*  control_panel is rviz plugins for provide basec functions.
*  It's create mission, save/load mission, send mission, send control system mode.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include <pluginlib/class_list_macros.h>
#include "control_panel/control_panel.h"

ControlPanel::ControlPanel(QWidget* parent):
    rviz::Panel(parent)
{
    // ROS init
    ros_thread = new RosThreadCP();
    ros_thread->init();

    // GUI init
    CreateWidgets();

    // Clear all old targets from marker server
    slotClearAllTargets();
    slotClearTrajectory();

    // Connect signals and slots
    connect(saveMissionButton,     SIGNAL(clicked()), this, SLOT(slotSaveMission()));
    connect(loadMissionButton,     SIGNAL(clicked()), this, SLOT(slotLoadMission()));
    connect(modeButton,            SIGNAL(clicked()), this, SLOT(slotSendMode()));
    connect(sendMissionButton,     SIGNAL(clicked()), this, SLOT(slotSendMission()));
    connect(addTargetButton,       SIGNAL(clicked()), this, SLOT(slotAddTarget()));
    connect(delTargetButton,       SIGNAL(clicked()), this, SLOT(slotDeleteTarget()));
    connect(clearTargetsButton,    SIGNAL(clicked()), this, SLOT(slotClearAllTargets()));
    connect(clearTrajectoryButton, SIGNAL(clicked()), this, SLOT(slotClearTrajectory()));
    connect(table,                 SIGNAL(signalUpdateTableTarget(ardrone_msgs::NavPose)),
                                                      this, SLOT(slotUpdateTableTarget(ardrone_msgs::NavPose)));
    connect(ros_thread,            SIGNAL(signalUpdateMarkerTarget(ardrone_msgs::NavPose)),
                                                      this, SLOT(slotUpdateMarkerTarget(ardrone_msgs::NavPose)),
                                                                                                Qt::DirectConnection );
}


// --- GUI interface functions

void ControlPanel::CreateWidgets()
{
    // Mode components init
    modeLabel = new QLabel(tr("Mode: "));

    modeComboBox = new QComboBox();
    modeComboBox->addItem(tr("Auto"));
    modeComboBox->addItem(tr("Manual"));
    modeComboBox->setCurrentIndex(-1);

    modeButton = new QPushButton(tr("Send mode"));
    modeButton->setFixedWidth(100);

    modeHLayout = new QHBoxLayout();
    modeHLayout->addWidget(modeLabel,    0, Qt::AlignTop);
    modeHLayout->addWidget(modeComboBox, 1, Qt::AlignTop);
    modeHLayout->addWidget(modeButton,   2, Qt::AlignTop);
    modeHLayout->addStretch(1000);

    // Fly mission table components
    table = new TargetsTable(this);

    // Fly mission table control buttons
    addTargetButton = new QPushButton(tr("+"));
    addTargetButton->setMaximumWidth(35);
    delTargetButton = new QPushButton(tr("-"));
    delTargetButton->setMaximumWidth(35);
    clearTargetsButton = new QPushButton(tr("Clear all targets"));
    clearTrajectoryButton = new QPushButton(tr("Clear trajectory"));

    ctrMissionTableHLayout = new QHBoxLayout();
    ctrMissionTableHLayout->addWidget(addTargetButton,       0, Qt::AlignLeft);
    ctrMissionTableHLayout->addWidget(delTargetButton,       1, Qt::AlignLeft);
    ctrMissionTableHLayout->addWidget(clearTargetsButton,    2, Qt::AlignLeft);
    ctrMissionTableHLayout->addWidget(clearTrajectoryButton, 3, Qt::AlignLeft);
    ctrMissionTableHLayout->addStretch(1000);

    // Fly mission set takeoff and land
    takeoffCheckBox = new QCheckBox(tr("Take off when start"));
    takeoffCheckBox->setToolTip(tr("Send command \"take off\" before execution of fly mission"));
    takeoffCheckBox->setFocusPolicy(Qt::NoFocus);

    landCheckBox = new QCheckBox(tr("Land after end"));
    landCheckBox->setToolTip(tr("Send command \"land\" after execution of fly mission"));
    landCheckBox->setFocusPolicy(Qt::NoFocus);

    checkBoxLayout = new QHBoxLayout;
    checkBoxLayout->addWidget(takeoffCheckBox);
    checkBoxLayout->addWidget(landCheckBox);

    // Fly mission control buttons
    sendMissionButton = new QPushButton(tr("Send mission"));
    sendMissionButton->setEnabled(false);
    sendMissionButton->setToolTip("Send mode for activate");
    loadMissionButton = new QPushButton(tr("Load"));
    saveMissionButton = new QPushButton(tr("Save"));

    ctrMissionHLayout = new QHBoxLayout();
    ctrMissionHLayout->addWidget(sendMissionButton, 0, Qt::AlignLeft);
    ctrMissionHLayout->addStretch(1000);
    ctrMissionHLayout->addWidget(loadMissionButton, 1, Qt::AlignRight);
    ctrMissionHLayout->addWidget(saveMissionButton, 2, Qt::AlignRight);

    // Add all in main layout
    mainVLayout = new QVBoxLayout();
    mainVLayout->addLayout(modeHLayout,            0);
    mainVLayout->addWidget(table,                  1);
    mainVLayout->addLayout(ctrMissionTableHLayout, 2);
    mainVLayout->addLayout(checkBoxLayout,         3);
    mainVLayout->addLayout(ctrMissionHLayout,      4);

    setLayout(mainVLayout);
}


// --- Slots

void ControlPanel::slotSendMode()
{
    ros_thread->SendMode(modeComboBox->currentIndex());

    sendMissionButton->setEnabled(true);
    sendMissionButton->setToolTip("");
}

void ControlPanel::slotSendMission()
{
    std::vector<ardrone_msgs::NavPose> mission;
    mission = table->GetMission();
    if(mission.empty())
        return;

    if(!CheckDroneSate()) // If drone state != flying
        return;

    ros_thread->SendMission(mission, takeoffCheckBox->isChecked(), landCheckBox->isChecked());
}

void ControlPanel::slotSaveMission()
{
    std::vector<ardrone_msgs::NavPose> mission = table->GetMission();
    if(mission.empty())
        return;

    QFileDialog fileDialog(0, tr("Save mission"), QDir::currentPath(), "MISSION(*.mission *.MISSION)");
    fileDialog.setFileMode(QFileDialog::AnyFile);
    fileDialog.setAcceptMode(QFileDialog::AcceptSave);
    fileDialog.selectFile("untitled.mission");

    QStringList strFileNames;
    if(fileDialog.exec())
        strFileNames = fileDialog.selectedFiles();

    if(strFileNames.isEmpty())
        return;

    if(!strFileNames.at(0).endsWith(".mission") && !strFileNames.at(0).endsWith(".mission"))
        strFileNames[0] += ".mission";

     QSettings * missionFile = new QSettings(strFileNames.at(0), QSettings::NativeFormat);

     missionFile->setValue("Cmd/take_off", takeoffCheckBox->isChecked());
     missionFile->setValue("Cmd/land",     landCheckBox->isChecked());

     for(int i = 0; i < mission.size(); i++) {
         QString strNum = QString::number(mission.at(i).num);
         missionFile->setValue(strNum + "/x",   mission.at(i).x);
         missionFile->setValue(strNum + "/y",   mission.at(i).y);
         missionFile->setValue(strNum + "/z",   mission.at(i).z);
     }

     missionFile->sync();
     delete missionFile;
}

void ControlPanel::slotLoadMission()
{
    if(table->GetCurTargetNum() > 0) {
        QMessageBox msgBox;
        msgBox.setIcon(msgBox.Question);
        msgBox.setText("The mission has been modified.");
        msgBox.setInformativeText("Do you want to save your changes?");
        msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Reset | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Save);
        int ret = msgBox.exec();

        switch (ret)
        {
        case QMessageBox::Save:
           slotSaveMission();
           slotClearAllTargets();
           break;
        case QMessageBox::Reset:
            slotClearAllTargets();
           break;
        case QMessageBox::Cancel:
           return;
        default:
           break;
        }
    }

    QFileDialog fileDialog(0, tr("Load mission"), QDir::currentPath(), "MISSION(*.mission *.MISSION)");
    fileDialog.setFileMode(QFileDialog::ExistingFile);
    fileDialog.setAcceptMode(QFileDialog::AcceptOpen);

    QStringList strFileNames;
    if(fileDialog.exec())
        strFileNames = fileDialog.selectedFiles();

    if(strFileNames.isEmpty())
        return;

    QSettings * missionFile = new QSettings(strFileNames.at(0), QSettings::NativeFormat);

    int i = 0;
    while(1) {
        if(!missionFile->contains(QString::number(i+1) + "/x"))
            break;

        ardrone_msgs::NavPose target;
        target.num = i+1;
        target.x   = missionFile->value(QString::number(i+1) + "/x",   "0").toDouble();
        target.y   = missionFile->value(QString::number(i+1) + "/y",   "0").toDouble();
        target.z   = missionFile->value(QString::number(i+1) + "/z",   "0").toDouble();

        if(!CheckTarget(target))
            return;

        table->AddTarget(target);

        ros_thread->SendAddTager(target.num);
        ros_thread->SendUpdateTarget(target);

        i++;
    }

    takeoffCheckBox->setChecked(missionFile->value("Cmd/take_off", "false").toBool());
    landCheckBox->setChecked(missionFile->value("Cmd/land",        "false").toBool());

    missionFile->sync();
    delete missionFile;
}

void ControlPanel::slotAddTarget()
{
    ardrone_msgs::NavPose newTarget;
    newTarget = ros_thread->SendAddTager((table->GetCurTargetNum()+1));

    table->AddTarget(newTarget);
}

void ControlPanel::slotDeleteTarget()
{
    int delTargetNum = table->GetCurTargetNum();

    if(ros_thread->SendDeleteTarget(delTargetNum))
        table->DeleteTarget(delTargetNum);
}

void ControlPanel::slotClearAllTargets()
{
    if(ros_thread->SendClearAllTargets())
        table->ClearAllTargets();
}

void ControlPanel::slotClearTrajectory()
{
    ros_thread->SendClearTrajectory();
}


void ControlPanel::slotUpdateMarkerTarget(ardrone_msgs::NavPose target)
{
    // Update marker from server to table
    target.num--;
    table->UpdateTarget(target);
}

void ControlPanel::slotUpdateTableTarget(ardrone_msgs::NavPose target)
{
    // Update marker from table to server
    ros_thread->SendUpdateTarget(target);
}


// --- Check data functions

bool ControlPanel::CheckTarget(const ardrone_msgs::NavPose &target)
{
    if( target.x   <= -100 || target.x   >= 100 ||
        target.y   <= -100 || target.y   >= 100 ||
        target.z   <=    0 || target.z   >= 100 ) {
        QMessageBox msgBox;
        msgBox.setIcon(msgBox.Warning);
        msgBox.setText("Coordinate value in the mission file is outside the permissible");
        msgBox.setStandardButtons(QMessageBox::Ok);

        slotClearAllTargets();
        return false;
    }

    return true;
}

bool ControlPanel::CheckDroneSate()
{
    // If drone state != flying
    if(ros_thread->GetDroneState() != 3 && ros_thread->GetDroneState() != 7 && !takeoffCheckBox->isChecked()) {
        QMessageBox msgBox;
        msgBox.setIcon(msgBox.Warning);
        msgBox.setText(QString("You send mission, but ardrone state is not equal \"Flying\".\n") +
                       QString("Probably ardrone is landed. Please, set flag \"Take off when start\" ") +
                       QString("and send mission again"));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();

        return false;
    }

    return true;
}


PLUGINLIB_EXPORT_CLASS(ControlPanel, rviz::Panel)
