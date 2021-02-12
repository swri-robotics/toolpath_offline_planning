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
#include "opp_gui/widgets/polyline_path_selection_widget.h"
#include "ui_polyline_path_selection_widget.h"

#include <QMessageBox>
#include <QPushButton>
#include <QCheckBox>

#include <std_srvs/Trigger.h>

namespace opp_gui
{
PolylinePathSelectionWidget::PolylinePathSelectionWidget(ros::NodeHandle& nh,
                                                         const std::string& selection_world_frame,
                                                         const std::string& selection_sensor_frame,
                                                         QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::PolylinePathSelectionWidget)
  , selector_(nh, selection_world_frame, selection_sensor_frame)
{
  ui_->setupUi(this);
  connect(ui_->push_button_clear_polyline, &QPushButton::clicked, this, &PolylinePathSelectionWidget::clearPolyline);
  connect(
      ui_->push_button_apply_polyline, &QPushButton::clicked, this, &PolylinePathSelectionWidget::applyPolylineAsPath);
  connect(ui_->push_button_htgen_polyline,
          &QPushButton::clicked,
          this,
          &PolylinePathSelectionWidget::applyPolylineforPathGen);
  connect(ui_->cbox_update_polyline, &QCheckBox::stateChanged, this, &PolylinePathSelectionWidget::updatePolyline);
  connect(this, &PolylinePathSelectionWidget::QWarningBox, this, &PolylinePathSelectionWidget::onQWarningBox);
  updatePolyline();  // synchronize checkboxes
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
    emit QWarningBox("Area Selection error: could not clear polygon points");
  }
  if (!srv.response.success)
  {
    std::string msg("Area Selection error:" + srv.response.message);
    emit QWarningBox(msg.c_str());
  }

  return;
}

void PolylinePathSelectionWidget::applyPolylineAsPath()
{
  if (!mesh_)
  {
    emit QWarningBox("No mesh available to crop");
    return;
  }

  std::string error_message;
  std::vector<int> path_indices;
  bool success = selector_.collectPathMesh(*mesh_, path_indices, error_message);
  if (!success)
  {
    std::string msg("Path Selection error: could not compute path: " + error_message);
    emit QWarningBox(msg.c_str());
  }
  else if (path_indices.size() == 0)
  {
    std::string msg("Path Selection error: no points found " + error_message);
    emit QWarningBox(msg.c_str());
  }
  emit(polylinePath(path_indices, mesh_));
  return;
}

void PolylinePathSelectionWidget::applyPolylineforPathGen()
{
  if (!mesh_)
  {
    emit QWarningBox("No mesh available to crop");
    return;
  }

  std::string error_message;
  std::vector<int> path_indices;
  bool success = selector_.collectPath(*mesh_, path_indices, error_message);
  if (!success)
  {
    std::string msg("Path Selection error: could not compute path: " + error_message);
    emit QWarningBox(msg.c_str());
  }
  // TODO perhaps we should send the mesh too, It seems like the other widget already knows which mesh has been
  // selected.
  emit(polylinePathGen(path_indices));

  return;
}

void PolylinePathSelectionWidget::updatePolyline()
{
  if (ui_->cbox_update_polyline->isChecked())
  {
    selector_.enable(true);
  }
  else
  {
    selector_.enable(false);
  }
}

void PolylinePathSelectionWidget::onQWarningBox(std::string warn_string)
{
  QMessageBox::warning(this, "Tool Path Planning Warning", QString(warn_string.c_str()));
}

}  // end namespace opp_gui
