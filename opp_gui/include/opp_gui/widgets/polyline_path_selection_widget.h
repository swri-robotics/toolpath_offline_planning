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

#ifndef OPP_GUI_WIDGETS_POLYLINE_PATH_SELECTION_WIDGET_H
#define OPP_GUI_WIDGETS_POLYLINE_PATH_SELECTION_WIDGET_H

#include <QWidget>

#include <ros/ros.h>
#include <shape_msgs/Mesh.h>

#include <opp_path_selection/path_selection_artist.h>
#include "opp_gui/register_ros_msgs_for_qt.h"

namespace Ui
{
class PolylinePathSelectionWidget;
}

namespace opp_gui
{
class PolylinePathSelectionWidget : public QWidget
{
  Q_OBJECT

public:
  explicit PolylinePathSelectionWidget(ros::NodeHandle& nh,
                                       const std::string& selection_world_frame,
                                       const std::string& selection_sensor_frame,
                                       QWidget* parent = nullptr);
  ~PolylinePathSelectionWidget();

public Q_SLOTS:
  void init(const shape_msgs::Mesh& mesh);

Q_SIGNALS:
  void polylinePath(const std::vector<int>& path_indices, const shape_msgs::Mesh::Ptr& mesh);

  void polylinePathGen(std::vector<int> pt_indices);

  void QWarningBox(std::string warn_string);

private Q_SLOTS:

  void clearPolyline();

  void applyPolylineAsPath();

  void applyPolylineforPathGen();

  void onQWarningBox(std::string warn_string);

  void updatePolyline();

private:
  Ui::PolylinePathSelectionWidget* ui_;

  shape_msgs::Mesh::Ptr mesh_;

  opp_path_selection::PathSelectionArtist selector_;

};  // end class PolylinePathSelectionWidget

}  // end namespace opp_gui

#endif  // OPP_GUI_WIDGETS_POLYLINE_PATH_SELECTION_WIDGET_H
