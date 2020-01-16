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

#ifndef OPP_GUI_WIDGETS_TOUCH_POINT_EDITOR_WIDGET_H
#define OPP_GUI_WIDGETS_TOUCH_POINT_EDITOR_WIDGET_H

#include <QWidget>

#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <std_msgs/ColorRGBA.h>

#include "opp_gui/widgets/list_editor_widget_base.h"
#include <opp_msgs/TouchPoint.h>

namespace Ui
{
class TouchPointParametersEditor;
}

namespace opp_gui
{
class TouchPointParametersEditorWidget;

/**
 * @brief A widget, based on the list editor widget, for editing the definition of multiple
 * touch points
 */
class TouchPointEditorWidget : public ListEditorWidgetBase
{
  Q_OBJECT
public:
  TouchPointEditorWidget(QWidget* parent = nullptr,
                         const ros::NodeHandle& nh = ros::NodeHandle("~"),
                         const std::string& marker_frame = "map");

  void setPoints(const std::vector<opp_msgs::TouchPoint>& points);

  inline std::map<std::string, opp_msgs::TouchPoint> getPoints() const { return data_; }

  inline void setMarkerFrame(const std::string& frame) { marker_frame_ = frame; }

  inline virtual void clear() override
  {
    ListEditorWidgetBase::clear();

    data_.clear();
  }

  void setMarkerColor(const double r, const double g, const double b, const double a = 1.0);

protected
  Q_SLOT :

    virtual void
    onAddPressed() override;

  virtual void onRemovePressed() override;

  virtual void onListSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous) override;

  virtual void onDataChanged() override;

private:
  TouchPointParametersEditorWidget* point_editor_;

  std::map<std::string, opp_msgs::TouchPoint> data_;

  std::string marker_frame_;

  std_msgs::ColorRGBA marker_color_;

  ros::NodeHandle nh_;

  ros::Publisher marker_pub_;
};

}  // namespace opp_gui

#endif  // OPP_GUI_WIDGETS_TOUCH_POINT_EDITOR_WIDGET_H
