/**
*  This file is part of ardrone_dcs.
*
*  targets_table is part of control panel rviz plugin.
*  It's provide table for create and corrected mission.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include "control_panel/targets_table.h"


TargetsTable::TargetsTable(QWidget *parent) :
    QWidget(parent)
{
    // Create table components
    tableModel = new QStandardItemModel(0, 3, this);
    tableDelegate = new TableDelegate(this);
    tableView = new QTableView(this);
    tableView->setModel(tableModel);
    tableView->setItemDelegate(tableDelegate);

    // Configurate table
    tableView->setSelectionMode(QAbstractItemView::NoSelection);
    tableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    QStringList headList;
    headList.append("x, m"); headList.append("y, m"); headList.append("z, m");
    tableModel->setHorizontalHeaderLabels(headList);
    tableView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);

    // Add table on layout
    tableLayout = new QGridLayout();
    tableLayout->addWidget(tableView);
    setLayout(tableLayout);

    connect(tableModel, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(slotTableTargetChanged(QStandardItem*)));
}


// --- Table interface functions

void TargetsTable::AddTarget(ardrone_msgs::NavPose target)
{
    target.num -= 1;
    tableModel->insertRow(target.num);

    UpdateTarget(target);
}

void TargetsTable::DeleteTarget(int num)
{
    num -= 1;
    if(num < 0)
        return;

    tableModel->removeRow(num);
}

void TargetsTable::ClearAllTargets()
{
    tableModel->removeRows(0, tableModel->rowCount());
}

void TargetsTable::UpdateTarget(ardrone_msgs::NavPose target)
{     
    tableModel->setData(tableModel->index(target.num, 0), round(target.x*1000)/1000.);
    tableModel->setData(tableModel->index(target.num, 1), round(target.y*1000)/1000.);
    tableModel->setData(tableModel->index(target.num, 2), round(target.z*1000)/1000.);
}

std::vector<ardrone_msgs::NavPose> TargetsTable::GetMission()
{
    std::vector<ardrone_msgs::NavPose> mission;
    for(int i = 0; i < tableModel->rowCount(); i++)
    {
        ardrone_msgs::NavPose target;
        target.num = i+1;
        target.x   = tableModel->data(tableModel->index(i, 0)).toDouble();
        target.y   = tableModel->data(tableModel->index(i, 1)).toDouble();
        target.z   = tableModel->data(tableModel->index(i, 2)).toDouble();

        mission.push_back(target);
    }

    return mission;
}

int TargetsTable::GetCurTargetNum()
{
    return tableModel->rowCount();
}


// --- Slots

void TargetsTable::slotTableTargetChanged(QStandardItem *item)
{
    int changedRow = item->index().row();

    ardrone_msgs::NavPose changedTarget;
    changedTarget.num = changedRow+1;
    changedTarget.x   = tableModel->data(tableModel->index(changedRow, 0)).toDouble();
    changedTarget.y   = tableModel->data(tableModel->index(changedRow, 1)).toDouble();
    changedTarget.z   = tableModel->data(tableModel->index(changedRow, 2)).toDouble();

    emit signalUpdateTableTarget(changedTarget);
}
