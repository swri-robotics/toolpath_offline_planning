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

#include "opp_gui/widgets/tool_path_editor_widget.h"
#include "opp_gui/widgets/tool_path_parameters_editor_widget.h"
#include <QMessageBox>

const static std::string TOOL_PATH_TOPIC = "tool_path";

namespace opp_gui
{
ToolPathEditorWidget::ToolPathEditorWidget(QWidget* parent,
                                           const ros::NodeHandle& nh,
                                           const std::string& marker_frame,
                                           const std::string& selection_world_frame,
                                           const std::string& selection_sensor_frame)
  : ListEditorWidgetBase(parent), nh_(nh), marker_frame_(marker_frame)
{
  surface_selector_ = new SurfaceSelectionComboWidget(nh_, selection_world_frame, selection_sensor_frame, this);
  editor_ = new ToolPathParametersEditorWidget(nh_, this);

  // Add the point editor to the parameters frame
  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(surface_selector_);
  layout->addWidget(editor_);
  ui_->frame_parameters->setLayout(layout);

  // Disable the parameters frame until a path is added
  ui_->frame_parameters->setEnabled(false);

  // Connect the signals and slots
  connect(ui_->push_button_add, &QPushButton::clicked, this, &ToolPathEditorWidget::onAddPressed);
  connect(ui_->push_button_remove, &QPushButton::clicked, this, &ToolPathEditorWidget::onRemovePressed);
  connect(ui_->list_widget, &QListWidget::currentItemChanged, this, &ToolPathEditorWidget::onListSelectionChanged);

  connect(editor_, &ToolPathParametersEditorWidget::dataChanged, this, &ToolPathEditorWidget::onDataChanged);

  connect(surface_selector_,
          &SurfaceSelectionComboWidget::newTargetMesh,
          this,
          &ToolPathEditorWidget::newTargetMeshSelected);
  // TODO what to do with a polyline created path
  connect(surface_selector_, &SurfaceSelectionComboWidget::polylinePath, this, &ToolPathEditorWidget::onPolylinePath);
  connect(surface_selector_,
          &SurfaceSelectionComboWidget::polylinePathGen,
          editor_,
          &ToolPathParametersEditorWidget::onPolylinePathGen);
  connect(this, &ToolPathEditorWidget::QWarningBox, this, &ToolPathEditorWidget::onQWarningBox);

  // Create a publisher for the tool path marker
  pub_ = nh_.advertise<geometry_msgs::PoseArray>(TOOL_PATH_TOPIC, 1, true);
}

void ToolPathEditorWidget::init(const shape_msgs::Mesh& mesh)
{
  surface_selector_->init(mesh);
  editor_->init(mesh);
}

void ToolPathEditorWidget::addToolPathData(const std::vector<opp_msgs::ToolPath>& tool_path_list)
{
  // reset the visualization
  opp_msgs::ToolPath empty;
  publishToolPathDisplay(empty);

  // empty the paths
  data_.clear();
  ui_->list_widget->clear();

  // Add the new paths to the list
  int i = 0;
  for (opp_msgs::ToolPath path : tool_path_list)
  {
    opp_msgs::ToolPath val;
    val = path;

    // If the config was empty, set default values. Floating-point
    // comparison to 0.0 will turn false negatives, but should never get
    // a false positive.  We definitely don't want to overwrite valid
    // parameters, so that seems OK for now.
    if (val.params.config.surface_walk_generator.raster_spacing == 0.0)
    {
      val.params.config.type = 0;  // SURFACE_WALK_RASTER_GENERATOR;
      val.params.config.surface_walk_generator.raster_spacing = 0.2;
      val.params.config.surface_walk_generator.point_spacing = 0.1;
      val.params.config.surface_walk_generator.min_hole_size = 0.2;
      val.params.config.surface_walk_generator.min_segment_size = 0.5;
      val.params.config.surface_walk_generator.intersection_plane_height = 0.05;

      val.params.curvature_threshold = 0.050;
      val.params.min_polygons_per_cluster = 500;
    }
    std::string key_base = "previous_path_";
    std::string key = key_base + std::to_string(i);
    while (data_.find(key) != data_.end())
    {
      ++i;
      if (i > 2000)
        break;  // can't be too careful
      key = key_base + std::to_string(i);
    }
    data_.emplace(key, val);
    ui_->list_widget->addItem(QString::fromStdString(key));
  }
}

void ToolPathEditorWidget::newTargetMeshSelected(const shape_msgs::Mesh::Ptr& target_mesh)
{
  if (target_mesh != nullptr)
  {
    editor_->init(*target_mesh);
  }
  return;
}

void ToolPathEditorWidget::onAddPressed()
{
  bool ok;
  QString key = QInputDialog::getText(this, "Add New", "Name", QLineEdit::Normal, "", &ok);
  if (ok && !key.isEmpty())
  {
    opp_msgs::ToolPath val;
    val.params.config.surface_walk_generator.raster_spacing = 0.2;
    val.params.config.surface_walk_generator.point_spacing = 0.1;
    val.params.config.surface_walk_generator.min_hole_size = 0.2;
    val.params.config.surface_walk_generator.min_segment_size = 0.5;
    val.params.config.surface_walk_generator.intersection_plane_height = 0.05;
    val.params.curvature_threshold = 0.050;
    val.params.min_polygons_per_cluster = 500;

    val.process_type.val = opp_msgs::ProcessType::NONE;

    // Make sure the key doesn't already exist in the map
    if (data_.find(key.toStdString()) == data_.end())
    {
      // Add the entry to the map of points
      data_.emplace(key.toStdString(), val);

      // Add the name to the list widget
      ui_->list_widget->addItem(key);
    }
  }
}

void ToolPathEditorWidget::onRemovePressed()
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
  }

  // Disable the parameters frame if there are no tool paths left
  if (data_.empty())
  {
    ui_->frame_parameters->setEnabled(false);
  }
}

void ToolPathEditorWidget::onListSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous)
{
  if (previous)
  {
    // Get the current data from the tool path editor widget and save it for the "previous" tool path
    opp_msgs::ToolPath tool_path;
    if (!editor_->getToolPath(tool_path))
    {
      ROS_WARN_STREAM(__func__ << ": Toolpath has not been generated yet");
      return;
    }
    tool_path.params.config = editor_->getToolPathConfig();
    tool_path.params.curvature_threshold = surface_selector_->getSegmentationConfig().curvature_threshold;
    tool_path.params.min_polygons_per_cluster = surface_selector_->getSegmentationConfig().min_cluster_size;

    std::string prev_name = previous->text().toStdString();
    auto prev_it = data_.find(prev_name);
    if (prev_it != data_.end())
    {
      prev_it->second = tool_path;
    }
    else
    {
      ROS_WARN_STREAM(__func__ << ": Previous point '" << prev_name << "' does not exist in the map");
    }
  }

  if (current)
  {
    // Enable the parameters frame if the current selection is valid
    ui_->frame_parameters->setEnabled(true);

    // Load the tool path configuration data for the "current" tool path into the point editor widget
    std::string curr_name = current->text().toStdString();
    auto curr_it = data_.find(curr_name);
    if (curr_it != data_.end())
    {
      // Set segmentation parameters
      noether_msgs::SegmentationConfig seg_config = surface_selector_->getSegmentationConfig();
      seg_config.min_cluster_size = curr_it->second.params.min_polygons_per_cluster;
      seg_config.curvature_threshold = curr_it->second.params.curvature_threshold;
      surface_selector_->setSegmentationConfig(seg_config);

      // Set rastering parameters
      editor_->setToolPathConfig(curr_it->second.params.config);

      // Set the tool path
      editor_->setToolPath(curr_it->second);

      // Display the current tool path
      opp_msgs::ToolPath tool_path;
      if (editor_->getToolPath(tool_path))
      {
        publishToolPathDisplay(tool_path);
      }
    }
    else
    {
      ROS_WARN_STREAM(__func__ << "Current point '" << curr_name << "' does not exist in the map");
    }
  }
}

void ToolPathEditorWidget::publishToolPathDisplay(const opp_msgs::ToolPath& tool_path)
{
  // Concatenate all of the tool path pose arrays (representing tool path segments) into one array for display
  geometry_msgs::PoseArray tool_path_display;
  for (const geometry_msgs::PoseArray& pose_arr : tool_path.paths)
  {
    tool_path_display.poses.insert(tool_path_display.poses.end(), pose_arr.poses.begin(), pose_arr.poses.end());
  }

  // Set the frame for this display
  tool_path_display.header.frame_id = marker_frame_;

  // Publish the display
  pub_.publish(tool_path_display);
}

void ToolPathEditorWidget::onPolylinePath(const std::vector<int> pnt_indices)
{
  // TODO: First find shortest path on surface between each segments same as with pathGen
  // TODO TODO TODO TODO
  char old_style[255];
  sprintf(old_style, "new polyline path in ToolPathEditor has %ld pnts", pnt_indices.size());
  emit QWarningBox(old_style);
}
void ToolPathEditorWidget::onPolylinePathReset(const std::vector<int> pnt_indices)
{
  std::string msg("Polyline path in ToolPathEditor was reset");
  emit QWarningBox(msg.c_str());
}

void ToolPathEditorWidget::onPolylinePathGen(const std::vector<int> pnt_indices)
{
  emit(editor_->polylinePathGen(pnt_indices));
}

void ToolPathEditorWidget::onPolylinePathGenReset(const std::vector<int> pnt_indices)
{
  emit(editor_->polylinePathGen(pnt_indices));
}

void ToolPathEditorWidget::onDataChanged()
{
  QListWidgetItem* current = ui_->list_widget->currentItem();
  if (current)
  {
    std::string key = current->text().toStdString();

    if (!key.empty())
    {
      auto it = data_.find(key);
      if (it != data_.end())
      {
        opp_msgs::ToolPath tool_path;
        if (editor_->getToolPath(tool_path))
        {
          if (surface_selector_ == nullptr)
          {
            emit QWarningBox("surface_selector_ not assigned in ToolPathEditorWidget");
          }
          else
          {
            tool_path.params.curvature_threshold = surface_selector_->getSegmentationConfig().curvature_threshold;
            tool_path.params.min_polygons_per_cluster = surface_selector_->getSegmentationConfig().min_cluster_size;
          }

          // Save the updated data internally
          it->second = tool_path;

          // Publish an updated tool path display
          publishToolPathDisplay(tool_path);
        }
      }
    }
  }
}

void ToolPathEditorWidget::onQWarningBox(std::string warn_string)
{
  QMessageBox::warning(this, "Tool Path Planning Warning", QString(warn_string.c_str()));
}

}  // namespace opp_gui
