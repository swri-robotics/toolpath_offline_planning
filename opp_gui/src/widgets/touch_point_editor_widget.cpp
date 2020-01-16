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

#include "opp_gui/widgets/touch_point_editor_widget.h"

#include <QInputDialog>

#include <eigen_conversions/eigen_msg.h>

#include "opp_gui/utils.h"
#include "opp_gui/widgets/touch_point_parameters_editor_widget.h"
#include "ui_touch_point_parameters_editor.h"

namespace opp_gui
{
TouchPointEditorWidget::TouchPointEditorWidget(QWidget* parent,
                                               const ros::NodeHandle& nh,
                                               const std::string& marker_frame)
  : ListEditorWidgetBase(parent), marker_frame_(marker_frame), marker_color_(utils::createColor(0.0, 0.0, 1.0)), nh_(nh)
{
  point_editor_ = new TouchPointParametersEditorWidget(this);

  // Add the point editor to the parameters frame
  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(point_editor_);
  ui_->frame_parameters->setLayout(layout);

  // Disable the editor until a point is added
  ui_->frame_parameters->setEnabled(false);

  // Set up the signals and slots
  // Add and remove buttons
  connect(ui_->push_button_add, &QPushButton::clicked, this, &TouchPointEditorWidget::onAddPressed);
  connect(ui_->push_button_remove, &QPushButton::clicked, this, &TouchPointEditorWidget::onRemovePressed);

  // Connect the point editor widget when list widget selection item changes
  connect(ui_->list_widget, &QListWidget::currentItemChanged, this, &TouchPointEditorWidget::onListSelectionChanged);

  connect(point_editor_, &TouchPointParametersEditorWidget::dataChanged, this, &TouchPointEditorWidget::onDataChanged);

  // Create a marker publisher
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("touch_point_marker", 5, true);
}

void TouchPointEditorWidget::onAddPressed()
{
  bool ok;
  QString key = QInputDialog::getText(this, "Add New", "Name", QLineEdit::Normal, "", &ok);
  if (ok && !key.isEmpty())
  {
    // Make sure the key doesn't already exist in the map
    if (data_.find(key.toStdString()) == data_.end())
    {
      // Set the default value of the touch point
      opp_msgs::TouchPoint tp;
      tp.name = key.toStdString();
      tp.transform.pose.orientation.w = 1.0;

      // Add the entry to the map of points
      data_.emplace(key.toStdString(), tp);

      // Add the name to the list widget
      ui_->list_widget->addItem(key);

      // Add and publish an arrow marker for this point
      visualization_msgs::Marker marker =
          utils::createArrowMarker(0, key.toStdString(), Eigen::Isometry3d::Identity(), marker_frame_, marker_color_);
      marker_pub_.publish(marker);
    }
  }
}

void TouchPointEditorWidget::onRemovePressed()
{
  // Get the selection from the list widget
  QList<QListWidgetItem*> current_items = ui_->list_widget->selectedItems();

  for (QListWidgetItem* item : current_items)
  {
    std::string key = item->text().toStdString();
    auto it = data_.find(key);
    if (it != data_.end())
    {
      data_.erase(it);
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << "Failed to find part '" << key << "' in map");
    }

    // Remove the selections from the list widget
    ui_->list_widget->removeItemWidget(item);
    delete item;

    // Remove the marker from the visualization
    visualization_msgs::Marker marker = utils::createArrowMarker(
        0, key, Eigen::Isometry3d::Identity(), marker_frame_, marker_color_, visualization_msgs::Marker::DELETE);
    marker_pub_.publish(marker);
  }

  // Disable the parameters frame if there are no touch points left
  if (data_.empty())
  {
    ui_->frame_parameters->setEnabled(false);
  }
}

void TouchPointEditorWidget::onListSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous)
{
  if (previous)
  {
    std::string prev_name = previous->text().toStdString();

    auto prev_it = data_.find(prev_name);
    if (prev_it != data_.end())
    {
      // Get the current data from the point editor widget and save it for the "previous" point
      opp_msgs::TouchPoint tp = point_editor_->getTouchPoint();

      // Overwrite the name with the correct value
      tp.name = prev_name;

      prev_it->second = tp;
    }
    else
    {
      ROS_WARN_STREAM(__func__ << ": Previous point '" << prev_name << "' does not exist in the map");
    }
  }

  if (current)
  {
    // Enable the parameters frame if the selection is valid
    ui_->frame_parameters->setEnabled(true);

    // Load the data for the "current" point into the point editor widget
    std::string curr_name = current->text().toStdString();
    auto curr_it = data_.find(curr_name);
    if (curr_it != data_.end())
    {
      point_editor_->setTouchPoint(curr_it->second);
    }
    else
    {
      ROS_WARN_STREAM(__func__ << "Current point '" << curr_name << "' does not exist in the map");
    }
  }
}

void TouchPointEditorWidget::onDataChanged()
{
  QListWidgetItem* item = ui_->list_widget->currentItem();
  if (item)
  {
    std::string key = item->text().toStdString();
    if (!key.empty())
    {
      // Save the updated information
      auto it = data_.find(key);
      if (it != data_.end())
      {
        // Get the current point from the editor
        opp_msgs::TouchPoint tp = point_editor_->getTouchPoint();

        // Overwrite the name with the current value
        tp.name = key;

        it->second = tp;

        // Publish an update marker for the point
        Eigen::Isometry3d pose;
        tf::poseMsgToEigen(tp.transform.pose, pose);

        visualization_msgs::Marker marker =
            utils::createArrowMarker(0, key, pose, marker_frame_, marker_color_, visualization_msgs::Marker::MODIFY);
        marker_pub_.publish(marker);
      }
    }
  }
}

void TouchPointEditorWidget::setPoints(const std::vector<opp_msgs::TouchPoint>& points)
{
  clear();

  for (const opp_msgs::TouchPoint& point : points)
  {
    // Add the entry to the map of points
    data_.emplace(point.name, point);

    // Add the name to the list widget
    ui_->list_widget->addItem(QString::fromStdString(point.name));
  }
}

void TouchPointEditorWidget::setMarkerColor(const double r, const double g, const double b, const double a)
{
  marker_color_ = utils::createColor(r, g, b, a);
}

}  // namespace opp_gui
