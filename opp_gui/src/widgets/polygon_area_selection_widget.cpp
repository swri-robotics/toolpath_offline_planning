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

#include "ui_polygon_area_selection_widget.h"

namespace opp_gui
{
PolygonAreaSelectionWidget::PolygonAreaSelectionWidget(ros::NodeHandle& nh,
                                                       const std::string& selection_world_frame,
                                                       const std::string& selection_sensor_frame,
                                                       QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::PolygonAreaSelectionWidget)
  , selector_(nh, selection_world_frame, selection_sensor_frame)
{
  ui_->setupUi(this);
  connect(
      ui_->push_button_clear_selection, &QPushButton::clicked, this, &PolygonAreaSelectionWidget::clearROISelection);
  connect(ui_->push_button_apply_selection, &QPushButton::clicked, this, &PolygonAreaSelectionWidget::applySelection);
  connect(ui_->cbox_update_selections, &QCheckBox::stateChanged, this, &PolygonAreaSelectionWidget::updateSelections);
  connect(this, &PolygonAreaSelectionWidget::QWarningBox, this, &PolygonAreaSelectionWidget::onQWarningBox);

  updateSelections();  // synchronize check boxes
}

PolygonAreaSelectionWidget::~PolygonAreaSelectionWidget() { delete ui_; }

void PolygonAreaSelectionWidget::init(const shape_msgs::Mesh& mesh)
{
  mesh_.reset(new shape_msgs::Mesh(mesh));
  clearROISelection();
  return;
}

void PolygonAreaSelectionWidget::clearROISelection()
{
  std_srvs::Trigger srv;
  // Currently, this callback is being called directly, so it will always return true.
  bool success = selector_.clearROIPointsCb(srv.request, srv.response);
  if (!success)
  {
    ROS_ERROR("Tool Path Parameter Editor Widget: Area Selection error: could not clear polygon points");
  }
  if (!srv.response.success)
  {
    ROS_ERROR_STREAM("Tool Path Parameter Editor Widget: Area Selection error:" << srv.response.message);
  }
  submesh_.reset(new shape_msgs::Mesh(*mesh_));

  emit(selectedSubmesh(submesh_));
  return;
}

void PolygonAreaSelectionWidget::applySelection()
{
  if (!mesh_)
  {
    emit QWarningBox("No mesh available to crop");
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

void PolygonAreaSelectionWidget::updateSelections()
{
  if (ui_->cbox_update_selections->isChecked())
  {
    selector_.enable(true);
  }
  else
  {
    selector_.enable(false);
  }
}

void PolygonAreaSelectionWidget::onQWarningBox(std::string warn_string)
{
  QMessageBox::warning(this, "Tool Path Planning Warning", QString(warn_string.c_str()));
}

}  // end namespace opp_gui
