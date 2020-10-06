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

#ifndef OPP_GUI_WIDGETS_SURFACE_SELECTION_COMBO_WIDGET_H
#define OPP_GUI_WIDGETS_SURFACE_SELECTION_COMBO_WIDGET_H

#include <QWidget>

#include <ros/ros.h>

#include <noether_msgs/SegmentationConfig.h>

#include "opp_gui/register_ros_msgs_for_qt.h"
#include "opp_gui/widgets/polygon_area_selection_widget.h"
#include "opp_gui/widgets/polyline_path_selection_widget.h"
#include "opp_gui/widgets/segmentation_parameters_editor_widget.h"

namespace Ui
{
class SurfaceSelectionComboWidget;
}

namespace opp_gui
{
class SurfaceSelectionComboWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SurfaceSelectionComboWidget(ros::NodeHandle& nh,
                                       const std::string& selection_world_frame,
                                       const std::string& selection_sensor_frame,
                                       QWidget* parent = nullptr);
  ~SurfaceSelectionComboWidget();

  void init(const shape_msgs::Mesh& mesh);

  /** @brief Create a segmentation config based on the spin boxes */
  noether_msgs::SegmentationConfig getSegmentationConfig();

  /** @brief Populates the spin boxes based on the segmentation config*/
  void setSegmentationConfig(const noether_msgs::SegmentationConfig& config);

Q_SIGNALS:
  void newTargetMesh(const shape_msgs::Mesh::Ptr& target_mesh);

  void polylinePath(const std::vector<int> point_indices);

  void polylinePathGen(const std::vector<int> pnt_indices);

private Q_SLOTS:
  void newSegmentList(const std::vector<shape_msgs::Mesh::Ptr>&, const shape_msgs::Mesh::Ptr&);

  void newSelectedSegment();

  void newSelectedSubmesh(const shape_msgs::Mesh::Ptr& selected_submesh);

  void onPolylinePath(const std::vector<int>& path_indices, const shape_msgs::Mesh::Ptr& mesh);

  void onPolylinePathGen(const std::vector<int>& pnt_indices);

private:
  void publishTargetMesh();

  Ui::SurfaceSelectionComboWidget* ui_;

  SegmentationParametersEditorWidget* segmenter_;

  PolygonAreaSelectionWidget* area_selector_;

  PolylinePathSelectionWidget* path_selector_;

  ros::Publisher selected_area_marker_publisher_;

  shape_msgs::Mesh::Ptr mesh_;

  std::vector<shape_msgs::Mesh::Ptr> segment_list_;

  shape_msgs::Mesh::Ptr selected_area_;
};

}  // end namespace opp_gui

#endif  // OPP_GUI_WIDGETS_SURFACE_SELECTION_COMBO_WIDGET_H
