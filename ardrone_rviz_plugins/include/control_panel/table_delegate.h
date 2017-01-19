/**
*  This file is part of ardrone_dcs.
*
*  table_delegate is part of control panel rviz plugin.
*  It's provide delegate for table with mission.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#pragma once
#ifndef TABLEDELEGATE_H
#define TABLEDELEGATE_H

#include <QtGui>
#include <QItemDelegate>
#include <QDoubleSpinBox>


class TableDelegate : public QItemDelegate
{
    Q_OBJECT
public:
    explicit TableDelegate(QObject *parent = 0);

    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &, const QModelIndex &index) const;
    void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;
    void setEditorData(QWidget *editor, const QModelIndex &index) const;

    void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const;
};

#endif // TABLEDELEGATE_H
