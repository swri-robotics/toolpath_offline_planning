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
  connect(ui_->push_button_apply_polyline, &QPushButton::clicked, this, &PolylinePathSelectionWidget::applyPolylineAsPath);
  connect(ui_->push_button_htgen_polyline, &QPushButton::clicked, this, &PolylinePathSelectionWidget::applyPolyline4PathGen);
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

  std::vector<int> bogus_pnts;
  emit(polylinePath(bogus_pnts, mesh_));
  emit(polylinePathGen(bogus_pnts));
  return;
}

void PolylinePathSelectionWidget::applyPolylineAsPath()
{
  if (!mesh_)
  {
    QMessageBox::warning(this, "Tool Path Planning Error", "No mesh available to crop");
    return;
  }

  std::string error_message;
  std::vector<int> path_indices;
  bool success = selector_.collectPathMesh(*mesh_, path_indices, error_message);
  if (!success)
  {
    ROS_ERROR_STREAM(
        "Tool Path Parameter Editor Widget: Path Selection error: could not compute path: " << error_message);
  }

  emit(polylinePath(path_indices, mesh_));
  return;
}

void PolylinePathSelectionWidget::applyPolyline4PathGen()
{
  if (!mesh_)
  {
    QMessageBox::warning(this, "Tool Path Planning Error", "No mesh available to crop");
    return;
  }

  std::string error_message;
  std::vector<int> path_indices;
  bool success = selector_.collectPath(*mesh_, path_indices, error_message);
  if (!success)
  {
    ROS_ERROR_STREAM(
        "Tool Path Parameter Editor Widget: Path Selection error: could not compute path: " << error_message);
  }

  // TODO perhaps we should send the mesh too, It seems like the other widget already knows which mesh has been selected. 
  emit(polylinePathGen(path_indices));

  return;
}

void PolylinePathSelectionWidget::writeMeshAsObj(const std::string& filename)
{
  ROS_ERROR("writing mesh to %s", filename.c_str());
  FILE *fp = fopen(filename.c_str(),"w");
  if(!fp)
    {
      ROS_ERROR("couldn't open file: %s", filename.c_str());
    }
  else
    {
      for(int i=0; i<mesh_->vertices.size(); i++)
	{
	  fprintf(fp, "v %lf %lf %lf\n", mesh_->vertices[i].x, mesh_->vertices[i].y, mesh_->vertices[i].z);
	}
      for(int i=0; i<mesh_->triangles.size(); i++)
	{
	  fprintf(fp, "f %ld %ld %ld\n",
		  mesh_->triangles[i].vertex_indices[0]+1,   // for some reason, .obj files are 1 referenced, not zero referenced
		  mesh_->triangles[i].vertex_indices[1]+1,
		  mesh_->triangles[i].vertex_indices[2]+1);
	}
    }
  fclose(fp);
}
void PolylinePathSelectionWidget::writePolylineAsSource(const std::string& filename, const std::vector<int>& path_indices)
{
  FILE *fp = fopen(filename.c_str(),"w");
  if(!fp)
    {
      ROS_ERROR("couldn't open file: %s", filename.c_str());
    }
  else
    {
      ROS_ERROR("source to %s", filename.c_str());
      fprintf(fp,"1\n%ld ", path_indices.size());
      for(int i=0; i<path_indices.size(); i++)
	{
	  fprintf(fp, "%ld ", path_indices[i]+1);
	}
      fprintf(fp,"\n");
    }
  fclose(fp);

}

}  // end namespace opp_gui
