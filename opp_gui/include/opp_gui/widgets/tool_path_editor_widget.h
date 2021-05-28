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

#ifndef OPP_GUI_WIDGETS_TOOLPATH_EDITOR_WIDGET_H
#define OPP_GUI_WIDGETS_TOOLPATH_EDITOR_WIDGET_H

#include <map>
#include <utility>  // pair

#include <noether_msgs/ToolPathConfig.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <shape_msgs/Mesh.h>

#include "opp_gui/widgets/list_editor_widget_base.h"
#include "opp_gui/widgets/surface_selection_combo_widget.h"
#include "opp_gui/widgets/tool_path_parameters_editor_widget.h"
#include <opp_msgs/ToolPath.h>

namespace opp_gui
{
typedef typename std::map<std::string, opp_msgs::ToolPath> ToolPathDataMap;

class ToolPathParametersEditorWidget;

/**
 * @brief A widget, based on the list editor base class widget, for editing the parameters of
 * and generating multiple tool paths
 */
class ToolPathEditorWidget : public ListEditorWidgetBase
{
  Q_OBJECT
public:
  ToolPathEditorWidget(QWidget* parent = nullptr,
                       const ros::NodeHandle& nh = ros::NodeHandle("~"),
                       const std::string& marker_frame = "map",
                       const std::string& selection_world_frame = "map",
                       const std::string& selection_sensor_frame = "map");

  inline virtual void clear() override
  {
    ListEditorWidgetBase::clear();

    data_.clear();
  }

  void init(const shape_msgs::Mesh& mesh);

  inline ToolPathDataMap getToolPathData() const { return data_; }

  void addToolPathData(const std::vector<opp_msgs::ToolPath>& tool_path_list);

  inline void setMarkerFrame(const std::string& frame) { marker_frame_ = frame; }

Q_SIGNALS:

  void polylinePathGen(const std::vector<int> pnt_indices);

  void QWarningBox(const std::string message);

protected Q_SLOTS:

  void newTargetMeshSelected(const shape_msgs::Mesh::Ptr& target_mesh);

  virtual void onAddPressed() override;

  virtual void onRemovePressed() override;

  virtual void onListSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous) override;

  virtual void onDataChanged() override;

  void onPolylinePath(const std::vector<int> pnt_indices);

  void onPolylinePathGen(const std::vector<int> pnt_indices);

  void onPolylinePathReset(const std::vector<int> pnt_indices);

  void onPolylinePathGenReset(const std::vector<int> pnt_indices);

  void onQWarningBox(const std::string message);

private:
  void publishToolPathDisplay(const opp_msgs::ToolPath& tool_path);

  SurfaceSelectionComboWidget* surface_selector_;

  ToolPathParametersEditorWidget* editor_;

  ToolPathDataMap data_;

  ros::NodeHandle nh_;

  ros::Publisher pub_;

  std::string marker_frame_;
};

}  // namespace opp_gui

#endif  // OPP_GUI_WIDGETS_TOOLPATH_EDITOR_WIDGET_H
