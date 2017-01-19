/**
*  This file is part of ardrone_dcs.
*
*  table_delegate is part of control panel rviz plugin.
*  It's provide delegate for table with mission.
*
*  Copyright 2016 Georgy Konovalov <konovalov.g.404@gmail.com> (SFEDU)
**/

#include "control_panel/table_delegate.h"
#include <iostream>

TableDelegate::TableDelegate(QObject *parent) :
    QItemDelegate(parent)
{
}

QWidget * TableDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem& /* option */,
                                                                                        const QModelIndex& index) const
{
    QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
    editor->setSingleStep(0.01);
    editor->setDecimals(3);

    switch (index.column())
    {
    case 0: // x
    case 1: // y
        editor->setMaximum(100);
        editor->setMinimum(-100);
        break;

    case 2: // z
        editor->setMaximum(100);
        editor->setMinimum(0);
        break;

    default:
        break;
    }

    return editor;
}

void TableDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex& index) const
{
    QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox*>(editor);
    model->setData(index, spinBox->value());
}

void TableDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
    double value = index.model()->data(index).toDouble();
    QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox*>(editor);
    spinBox->setValue(value);
}

void TableDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex& /* index */) const
{
    editor->setGeometry(option.rect);
}
