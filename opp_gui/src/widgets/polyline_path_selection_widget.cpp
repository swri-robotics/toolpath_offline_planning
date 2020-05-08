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

#include "opp_gui/widgets/polygon_area_selection_widget.h"

#include <QMessageBox>
#include <QPushButton>

#include <std_srvs/Trigger.h>

#include "ui_polyline_path_selection_widget.h"

namespace opp_gui
{
PolylinPathSelectionWidget::PolylinPathSelectionWidget(ros::NodeHandle& nh,
                                                       const std::string& selection_world_frame,
                                                       const std::string& selection_sensor_frame,
                                                       QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::PolygonAreaSelectionWidget)
  , selector_(nh, selection_world_frame, selection_sensor_frame)
{
  ui_->setupUi(this);
  connect(ui_->push_button_clear_polyline, &QPushButton::clicked, this, &PolygonAreaSelectionWidget::clearPolyline);
  connect(ui_->push_button_apply_polyline, &QPushButton::clicked, this, &PolygonAreaSelectionWidget::applyPolyline);
}

PolylinePathSelectionWidget::~PolylinePathSelectionWidget() { delete ui_; }

void PolylinePathSelectionWidget::init(const shape_msgs::Mesh& mesh)
{
  mesh_.reset(new shape_msgs::Mesh(mesh));
  clearPolyline();
  return;
}

void PolylinePathSelectionWidget::clearPolyline()
{
  std_srvs::Trigger srv;
  // Currently, this callback is being called directly, so it will always return true.
  bool success = selector_.clearPathPointsCb(srv.request, srv.response);
  if (!success)
  {
    ROS_ERROR("Tool Path Parameter Editor Widget: Area Selection error: could not clear polygon points");
  }
  if (!srv.response.success)
  {
    ROS_ERROR_STREAM("Tool Path Parameter Editor Widget: Area Selection error:" << srv.response.message);
  }
  submesh_.reset(new shape_msgs::Mesh(*mesh_));

  emit(selectedPath(submesh_));
  return;
}

void PolylinePathSelectionWidget::applyPolyline()
{
  if (!mesh_)
  {
    QMessageBox::warning(this, "Tool Path Planning Error", "No mesh available to crop");
    return;
  }
  submesh_.reset(new shape_msgs::Mesh());
  std::string error_message;
  bool success = selector_.collectROIMesh(*mesh_, *submesh_, error_message);
  if (!success)
  {
    ROS_ERROR_STREAM(
        "Tool Path Parameter Editor Widget: Area Selection error: could not compute submesh: " << error_message);
  }
  if (submesh_->vertices.size() < 3 || submesh_->triangles.size() < 1)
  {
    submesh_.reset(new shape_msgs::Mesh(*mesh_));
  }

  emit(selectedSubmesh(submesh_));
  return;
}

}  // end namespace opp_gui
