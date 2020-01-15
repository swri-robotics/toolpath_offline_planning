/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OPP_GUI_WIDGETS_LIST_EDITOR_WIDGET_BASE_H
#define OPP_GUI_WIDGETS_LIST_EDITOR_WIDGET_BASE_H

#include <QWidget>
#include <ui_list_editor.h>
#include <QInputDialog>

namespace opp_gui
{
/**
 * @brief Base class widget for editing the parameters of a list of objects. The UI for this
 * widget has a `QListWidget` display on the left to enumerate the objects and an empty panel
 * on the right in which to load a widget to display the parameters of the selection
 */
class ListEditorWidgetBase : public QWidget
{
  Q_OBJECT
public:
  ListEditorWidgetBase(QWidget* parent = nullptr) : QWidget(parent)
  {
    ui_ = new Ui::ListEditor();
    ui_->setupUi(this);
  }

  virtual ~ListEditorWidgetBase() {}

  /**
   * @brief Clears the entries in the `QListWidget` display
   */
  virtual void clear() { ui_->list_widget->clear(); }

protected Q_SLOTS:

  virtual void onAddPressed() = 0;

  virtual void onRemovePressed() = 0;

  virtual void onListSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous) = 0;

  virtual void onDataChanged() = 0;

protected:
  Ui::ListEditor* ui_;
};

}  // namespace opp_gui

#endif  // OPP_GUI_WIDGETS_LIST_EDITOR_WIDGET_BASE_H
