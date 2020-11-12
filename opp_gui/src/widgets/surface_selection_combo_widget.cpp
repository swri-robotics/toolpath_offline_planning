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

#include "opp_gui/widgets/surface_selection_combo_widget.h"

#include <QMessageBox>
#include <QMetaType>

#include <visualization_msgs/Marker.h>

#include "opp_gui/widgets/polygon_area_selection_widget.h"
#include "opp_gui/widgets/polyline_path_selection_widget.h"
#include "opp_gui/widgets/segmentation_parameters_editor_widget.h"
#include "ui_surface_selection_combo_widget.h"

namespace opp_gui
{
const static std::string MARKER_TOPIC = "/target_area";

SurfaceSelectionComboWidget::SurfaceSelectionComboWidget(ros::NodeHandle& nh,
                                                         const std::string& selection_world_frame,
                                                         const std::string& selection_sensor_frame,
                                                         QWidget* parent)
  : QWidget(parent)                           // call the base-class constructor
  , ui_(new Ui::SurfaceSelectionComboWidget)  // initialize the visible element
{
  // Setup the publisher for visualizing the selected area in RViz
  selected_area_marker_publisher_ = nh.advertise<visualization_msgs::Marker>(MARKER_TOPIC, 1, true);

  // Register ROS-message types with QT so we can use them in signals & slots
  int id = qRegisterMetaType<shape_msgs::Mesh>();
  id = qRegisterMetaType<shape_msgs::Mesh::Ptr>();
  id = qRegisterMetaType<std::vector<shape_msgs::Mesh::Ptr>>();

  // Initialize the visual component
  ui_->setupUi(this);

  // Create and attach the segmentation widget
  segmenter_ = new opp_gui::SegmentationParametersEditorWidget(this);
  ui_->layout_for_segmenter_widget->addWidget(segmenter_);

  // Set behavior on the list selection widget (which was initialized as part of ui_)
  ui_->list_widget_segment_list->setSelectionBehavior(QAbstractItemView::SelectItems);
  ui_->list_widget_segment_list->setSelectionMode(QAbstractItemView::SingleSelection);
  ui_->list_widget_segment_list->addItem("Full Mesh");

  // Create and attach the area selection widget
  area_selector_ = new opp_gui::PolygonAreaSelectionWidget(nh, selection_world_frame, selection_sensor_frame, this);
  ui_->layout_for_selector_widget->addWidget(area_selector_);

  path_selector_ = new opp_gui::PolylinePathSelectionWidget(nh, selection_world_frame, selection_sensor_frame, this);
  ui_->layout_for_selector_widget->addWidget(path_selector_);

  // Connect the inputs and outputs of sub-widgets
  connect(segmenter_,
          &opp_gui::SegmentationParametersEditorWidget::segmentationFinished,
          this,
          &opp_gui::SurfaceSelectionComboWidget::newSegmentList);

  connect(ui_->list_widget_segment_list,
          &QListWidget::itemSelectionChanged,
          this,
          &opp_gui::SurfaceSelectionComboWidget::newSelectedSegment);

  connect(area_selector_,
          &PolygonAreaSelectionWidget::selectedSubmesh,
          this,
          &SurfaceSelectionComboWidget::newSelectedSubmesh);

  connect(
      path_selector_, &PolylinePathSelectionWidget::polylinePath, this, &SurfaceSelectionComboWidget::onPolylinePath);

  connect(path_selector_,
          &PolylinePathSelectionWidget::polylinePathGen,
          this,
          &SurfaceSelectionComboWidget::onPolylinePathGen);
}

SurfaceSelectionComboWidget::~SurfaceSelectionComboWidget()
{
  delete ui_;
  delete segmenter_;
  delete area_selector_;
  delete path_selector_;
}

noether_msgs::SegmentationConfig SurfaceSelectionComboWidget::getSegmentationConfig()
{
  noether_msgs::SegmentationConfig config;
  if (segmenter_ == nullptr)
  {
    ROS_ERROR("segmenter_ not set in SurfaceSelectionComboWidget");
  }
  else
  {
    config = segmenter_->getSegmentationConfig();
  }
  return config;
}

void SurfaceSelectionComboWidget::setSegmentationConfig(const noether_msgs::SegmentationConfig& config)
{
  if (segmenter_ == nullptr)
  {
    ROS_ERROR("segmenter_ not set in SurfaceSelectionComboWidget");
  }
  else
  {
    segmenter_->setSegmentationConfig(config);
  }
  return;
}

void SurfaceSelectionComboWidget::init(const shape_msgs::Mesh& mesh)
{
  // Set the mesh
  mesh_.reset(new shape_msgs::Mesh(mesh));

  // Empty the segment list, and reset the 'whole mesh' option
  segment_list_.resize(1);
  segment_list_[0].reset(new shape_msgs::Mesh(mesh));
  ui_->list_widget_segment_list->clear();
  ui_->list_widget_segment_list->addItem("Full Mesh");
  ui_->list_widget_segment_list->setCurrentRow(0);

  // Reset the selected area
  selected_area_.reset(new shape_msgs::Mesh(mesh));

  // Send the mesh along to the segmentation widget
  // (The polygon selector will get it automatically)
  segmenter_->init(mesh);
  return;
}

void SurfaceSelectionComboWidget::newSegmentList(const std::vector<shape_msgs::Mesh::Ptr>& segments,
                                                 const shape_msgs::Mesh::Ptr& remnants)
{
  // Save the original mesh with the segments
  segment_list_.resize(1);
  segment_list_[0].reset(new shape_msgs::Mesh(*mesh_));
  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    segment_list_.emplace_back(new shape_msgs::Mesh(*(segments[i])));
  }

  // Add labels for each segment to the list
  ui_->list_widget_segment_list->clear();
  ui_->list_widget_segment_list->addItem("Full Mesh");
  std::string label;
  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    label = "segment_" + boost::lexical_cast<std::string>(i);
    ui_->list_widget_segment_list->addItem(label.c_str());
  }

  return;
}

void SurfaceSelectionComboWidget::newSelectedSegment()
{
  // Get the selected item(s) from the list
  QList<QListWidgetItem*> selectedSegments = ui_->list_widget_segment_list->selectedItems();

  // Make sure that there is a segment selected, otherwise crashes can happen
  if (selectedSegments.size() == 0)
  {
    return;
  }

  // Since the listWidget was set to have only one selection in this
  // class's constructor, we just use the first item.
  std::size_t selection_index = static_cast<std::size_t>(ui_->list_widget_segment_list->currentRow());

  // Based on the user's selection, send the appropriate segment to the
  // polygon selection tool and set the selected area to the entire
  // selected segment.
  selected_area_.reset(new shape_msgs::Mesh(*(segment_list_[selection_index])));
  area_selector_->init(*(segment_list_[selection_index]));
  path_selector_->init(*(segment_list_[selection_index]));
  return;
}

void SurfaceSelectionComboWidget::newSelectedSubmesh(const shape_msgs::Mesh::Ptr& selected_submesh)
{
  // Save the selected submesh, display it, then emit it as a signal
  selected_area_.reset(new shape_msgs::Mesh(*selected_submesh));
  publishTargetMesh();
  emit newTargetMesh(selected_area_);

  return;
}

void SurfaceSelectionComboWidget::onPolylinePath(const std::vector<int>& path_indices,
                                                 const shape_msgs::Mesh::Ptr& mesh)
{
  emit polylinePath(path_indices);
  return;
}

void SurfaceSelectionComboWidget::onPolylinePathGen(const std::vector<int>& path_indices)
{
  emit polylinePathGen(path_indices);
  return;
}

void SurfaceSelectionComboWidget::publishTargetMesh()
{
  visualization_msgs::Marker target_mesh;
  target_mesh.header.frame_id = "map";
  target_mesh.header.stamp = ros::Time();
  target_mesh.id = 0;
  target_mesh.type = visualization_msgs::Marker::TRIANGLE_LIST;
  target_mesh.action = visualization_msgs::Marker::ADD;
  target_mesh.pose.position.x = 0;
  target_mesh.pose.position.y = 0;
  target_mesh.pose.position.z = 0;
  target_mesh.pose.orientation.w = 1;
  target_mesh.pose.orientation.x = 0;
  target_mesh.pose.orientation.y = 0;
  target_mesh.pose.orientation.z = 0;
  target_mesh.scale.x = 1;
  target_mesh.scale.y = 1;
  target_mesh.scale.z = 1;
  target_mesh.color.a = 1;
  target_mesh.color.r = 1;
  target_mesh.color.b = 0;
  target_mesh.color.g = 1;
  target_mesh.points.clear();
  for (std::size_t i = 0; i < selected_area_->triangles.size(); ++i)
  {
    target_mesh.points.push_back(selected_area_->vertices[selected_area_->triangles[i].vertex_indices[0]]);
    target_mesh.points.push_back(selected_area_->vertices[selected_area_->triangles[i].vertex_indices[1]]);
    target_mesh.points.push_back(selected_area_->vertices[selected_area_->triangles[i].vertex_indices[2]]);
  }
  selected_area_marker_publisher_.publish(target_mesh);
  return;
}

}  // end namespace opp_gui
