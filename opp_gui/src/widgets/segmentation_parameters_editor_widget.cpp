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

#include "opp_gui/widgets/segmentation_parameters_editor_widget.h"

#include <pcl/io/vtk_lib_io.h>
#include <QProgressDialog>
#include <QMessageBox>

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include "opp_gui/utils.h"
#include "ui_segmentation_parameters_editor.h"

const static std::string SEGMENTATION_ACTION = "/mesh_segmenter_server_node/segmenter/";

namespace opp_gui
{
SegmentationParametersEditorWidget::SegmentationParametersEditorWidget(QWidget* parent)
  : QWidget(parent), client_(SEGMENTATION_ACTION, true)
{
  ui_ = new Ui::SegmentationParametersEditor();
  ui_->setupUi(this);

  // Parameter boxes
  ui_->spin_box_min_cluster_size->setRange(0, std::numeric_limits<int>::max());
  ui_->double_spin_box_curvature_threshold->setRange(0.0, 1.0);

  // Connect the button press to the function that calls the toolpath generation action
  connect(ui_->push_button_segment, &QPushButton::clicked, this, &SegmentationParametersEditorWidget::segmentMesh);
}

void SegmentationParametersEditorWidget::init(const pcl_msgs::PolygonMesh& mesh)
{
  shape_msgs::Mesh input_mesh;
  opp_gui::utils::pclMsgToShapeMsg(mesh, input_mesh);
  input_mesh_.reset(new shape_msgs::Mesh(input_mesh));
}

void SegmentationParametersEditorWidget::init(const shape_msgs::Mesh& mesh)
{
  input_mesh_.reset(new shape_msgs::Mesh(mesh));
}

void SegmentationParametersEditorWidget::setSegmentationConfig(const noether_msgs::SegmentationConfig& config)
{
  ROS_INFO("Setting Segmentation Parameters");

  ui_->spin_box_min_cluster_size->setValue(config.min_cluster_size);
  ui_->double_spin_box_curvature_threshold->setValue(config.curvature_threshold);
}

void SegmentationParametersEditorWidget::setFilteringConfig(const noether_msgs::FilteringConfig& config)
{
  ROS_INFO("Setting Filtering Parameters");
  // This is currently empty
}

noether_msgs::SegmentationConfig SegmentationParametersEditorWidget::getSegmentationConfig() const
{
  noether_msgs::SegmentationConfig config;

  // Create a path configuration from the line edit fields
  config.min_cluster_size = ui_->spin_box_min_cluster_size->value();
  config.curvature_threshold = ui_->double_spin_box_curvature_threshold->value();

  // TODO: Advanced setting
  config.use_mesh_normals = true;
  // If false enable box
  //  config.neighborhood_radius;

  return config;
}

noether_msgs::FilteringConfig SegmentationParametersEditorWidget::getFilteringConfig() const
{
  noether_msgs::FilteringConfig config;

  // Should add a dropdown for advanced settings
  config.enable_filtering = true;
  config.windowed_sinc_iterations = 20;

  return config;
}

void SegmentationParametersEditorWidget::segmentMesh()
{
  ROS_INFO("Performing Segmentation");

  if (!client_.isServerConnected())
  {
    std::string message = "Action server on '" + SEGMENTATION_ACTION + "' is not connected";
    QMessageBox::warning(this, "ROS Communication Error", QString(message.c_str()));
    return;
  }

  if (!input_mesh_)
  {
    QMessageBox::warning(this, "Input Error", "Mesh has not yet been specified");
    return;
  }

  // Get the correct mesh format for Noether
  pcl_msgs::PolygonMesh pcl_mesh;
  opp_gui::utils::pclMsgFromShapeMsg(*input_mesh_, pcl_mesh);

  // Create an action goal
  noether_msgs::SegmentGoal goal;
  goal.input_mesh = std::move(pcl_mesh);
  goal.filtering_config = getFilteringConfig();
  goal.segmentation_config = getSegmentationConfig();

  client_.sendGoal(goal, boost::bind(&SegmentationParametersEditorWidget::onSegmentMeshComplete, this, _1, _2));

  progress_dialog_ = new QProgressDialog(this);
  progress_dialog_->setModal(true);
  progress_dialog_->setLabelText("Segmentation Progress");
  progress_dialog_->setMinimum(0);
  progress_dialog_->setMaximum(100);

  progress_dialog_->setValue(progress_dialog_->minimum());
  progress_dialog_->show();
}

void SegmentationParametersEditorWidget::onSegmentMeshComplete(const actionlib::SimpleClientGoalState& state,
                                                               const noether_msgs::SegmentResultConstPtr& res)
{
  // TODO: Actually make this do something useful
  for (int i = progress_dialog_->minimum(); i < progress_dialog_->maximum(); ++i)
  {
    progress_dialog_->setValue(i);
    ros::Duration(0.01).sleep();
  }
  progress_dialog_->hide();

  if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std::string message = "Action '" + SEGMENTATION_ACTION + "' failed to succeed";
    QMessageBox::warning(this, "Segmentation Error", QString(message.c_str()));
  }
  else
  {
    ROS_INFO_STREAM("Successfully segmented the mesh");

    // Store results
    segments_.resize(res->output_mesh.size() - 1);
    shape_msgs::Mesh mesh_msg;
    for (std::size_t ind = 0; ind < res->output_mesh.size() - 1; ind++)
    {
      // get the pcl_msgs::PolygonMesh
      res->output_mesh[ind];

      // convert to shape_msgs::Mesh
      opp_gui::utils::pclMsgToShapeMsg(res->output_mesh[ind], mesh_msg);

      // Add to array
      segments_[ind].reset(new shape_msgs::Mesh(mesh_msg));

      // convert to pcl::PolygonMesh
      pcl::PolygonMesh pcl_mesh;
      pcl_conversions::toPCL(res->output_mesh[ind], pcl_mesh);

      // Save to file
      std::string filename = "/tmp/segment_" + boost::lexical_cast<std::string>(ind) + ".stl";
      pcl::io::savePolygonFile(filename, pcl_mesh, true);
    }
    opp_gui::utils::pclMsgToShapeMsg(res->output_mesh.back(), mesh_msg);
    edges_.reset(new shape_msgs::Mesh(mesh_msg));

    emit segmentationFinished(segments_, edges_);
  }
}

}  // namespace opp_gui
