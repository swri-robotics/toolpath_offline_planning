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

#ifndef OPP_GUI_WIDGETS_SEGMENTATION_PARAMETERS_EDITOR_WIDGET_H
#define OPP_GUI_WIDGETS_SEGMENTATION_PARAMETERS_EDITOR_WIDGET_H

#include <QWidget>

#include <actionlib/client/simple_action_client.h>
#include <noether_msgs/SegmentAction.h>
#include <shape_msgs/Mesh.h>

#include "opp_gui/register_ros_msgs_for_qt.h"

namespace Ui
{
class SegmentationParametersEditor;
}

class QProgressDialog;

namespace opp_gui
{
/**
 * @brief A widget for segmenting a mesh into its constituent surfaces based on curvature
 */
class SegmentationParametersEditorWidget : public QWidget
{
  Q_OBJECT
public:
  SegmentationParametersEditorWidget(QWidget* parent = nullptr);

  /** @brief initialize the widget with the input mesh */
  void init(const pcl_msgs::PolygonMesh& mesh);

  /** @brief initialize the widget with the input mesh */
  void init(const shape_msgs::Mesh& mesh);

  /** @brief Populates the spin boxes based on the segmentation config*/
  void setSegmentationConfig(const noether_msgs::SegmentationConfig& config);

  /** @brief Populates the spin boxes based on the filtering config*/
  void setFilteringConfig(const noether_msgs::FilteringConfig& config);

  /** @brief Create a segmentation config based on the spin boxes */
  noether_msgs::SegmentationConfig getSegmentationConfig() const;

  /** @brief Create a Filtering config based on the spin boxes */
  noether_msgs::FilteringConfig getFilteringConfig() const;

  /** @brief Returns the segmentation results as a vector*/
  std::vector<shape_msgs::Mesh::Ptr> getSegments() const { return segments_; }

  /** @brief Return the segmentation edges. These are the mesh trianges that were not classified into another segment */
  shape_msgs::Mesh::Ptr getEdges() const { return edges_; }

Q_SIGNALS:

  /**
   * @brief Signal emitted when segmentation is finished containing vector of the segments and the edges
   */
  void segmentationFinished(const std::vector<shape_msgs::Mesh::Ptr>&, const shape_msgs::Mesh::Ptr&);

private Q_SLOTS:

  /** @brief Performs segmentation by calling the segmentation action */
  void segmentMesh();

private:
  void onSegmentMeshComplete(const actionlib::SimpleClientGoalState& state,
                             const noether_msgs::SegmentResultConstPtr& res);

  actionlib::SimpleActionClient<noether_msgs::SegmentAction> client_;

  Ui::SegmentationParametersEditor* ui_;

  shape_msgs::Mesh::Ptr input_mesh_;

  std::vector<shape_msgs::Mesh::Ptr> segments_;

  shape_msgs::Mesh::Ptr edges_;

  QProgressDialog* progress_dialog_;
};

}  // namespace opp_gui

#endif  // OPP_GUI_WIDGETS_TOOL_PATH_PARAMETERS_EDITOR_WIDGET_H
