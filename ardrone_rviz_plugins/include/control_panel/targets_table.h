/**
*  This file is part of ardrone_dcs.
*
*  targets_table is part of control panel rviz plugin.
*  It's provide table for create and corrected mission.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#pragma once
#ifndef TARGETSTABLE_H
#define TARGETSTABLE_H

#include <QWidget>
#include <QTableView>
#include <QTableWidgetItem>
#include <QItemDelegate>
#include <QItemSelectionModel>
#include <QStandardItemModel>

#include "ardrone_msgs/NavPose.h"
#include "control_panel/table_delegate.h"

class TargetsTable : public QWidget
{
    Q_OBJECT
public:
    explicit TargetsTable(QWidget *parent = 0);

    // --- Table interface functions
    void AddTarget(ardrone_msgs::NavPose target);
    void DeleteTarget(int num);
    void ClearAllTargets();
    void UpdateTarget(ardrone_msgs::NavPose target);
    int GetCurTargetNum();
    std::vector<ardrone_msgs::NavPose> GetMission();

private:
    QTableView * tableView;
    QStandardItemModel * tableModel;
    TableDelegate * tableDelegate;
    QGridLayout * tableLayout;

private slots:
    void slotTableTargetChanged(QStandardItem* item);

signals:
    void signalUpdateTableTarget(ardrone_msgs::NavPose target);
};

#endif // TARGETSTABLE_H
