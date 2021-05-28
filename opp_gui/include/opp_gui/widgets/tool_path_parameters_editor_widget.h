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

#ifndef OPP_GUI_WIDGETS_TOOL_PATH_PARAMETERS_EDITOR_WIDGET_H
#define OPP_GUI_WIDGETS_TOOL_PATH_PARAMETERS_EDITOR_WIDGET_H

#include <QWidget>

#include <actionlib/client/simple_action_client.h>
#include <noether_msgs/GenerateToolPathsAction.h>
#include <heat_msgs/GenerateHeatToolPathsAction.h>
#include <heat_msgs/HeatRasterGeneratorConfig.h>
#include <ros/ros.h>

#include <opp_msgs/ToolPath.h>

namespace Ui
{
class ToolPathParametersEditor;
}

class QProgressDialog;

namespace opp_gui
{
/**
 * @brief A widget for editing the parameters associated with a single tool path, and generating
 * a tool path on a specified mesh based on those parameters
 */
class ToolPathParametersEditorWidget : public QWidget
{
  Q_OBJECT

public:
  /**
   * @brief constructor
   **/
  ToolPathParametersEditorWidget(ros::NodeHandle& nh, QWidget* parent = nullptr);

  /**
   * @brief Sets the internal mesh to be used for path planning and a parameter (not intended to
   * be user-facing) that defines the tool path generation process
   * @param mesh
   * @param intersecting_plane_height
   */
  void init(const shape_msgs::Mesh& mesh);

  void setToolPath(const opp_msgs::ToolPath& tool_path);

  bool getToolPath(opp_msgs::ToolPath& tool_path) const;

  void setToolPathConfig(const noether_msgs::ToolPathConfig& config);

  noether_msgs::ToolPathConfig getToolPathConfig() const;

  heat_msgs::HeatRasterGeneratorConfig getHeatRasterGeneratorConfig() const;

Q_SIGNALS:

  /**
   * @brief Signal emitted when data has changed in one of the editable UI fields
   */
  void dataChanged();

  // signal emitted when user selects a polyline they want as a path
  void polylinePath(const std::vector<int> pnt_indices);

  // signal emitted when user selects a polyline they want to use as a heat source to generate a path
  void polylinePathGen(const std::vector<int> pnt_indices);

  // signal emitted when ros thread wants a warning box
  void QWarningBox(std::string warn_string);

private Q_SLOTS:

  void updateProcessType(const QString&);

  void updateDwellTime(int value);

  void generateToolPath();

public Q_SLOTS:

  void onPolylinePath(const std::vector<int> pnt_indices);

  void onPolylinePathGen(const std::vector<int> pnt_indices);

private:
  void onGenerateToolPathsComplete(const actionlib::SimpleClientGoalState& state,
                                   const noether_msgs::GenerateToolPathsResultConstPtr& res);

  void onGenerateHeatToolPathsComplete(const actionlib::SimpleClientGoalState& state,
                                       const heat_msgs::GenerateHeatToolPathsResultConstPtr& res);

  void onQWarningBox(std::string warn_string);

  actionlib::SimpleActionClient<noether_msgs::GenerateToolPathsAction> client_;
  actionlib::SimpleActionClient<heat_msgs::GenerateHeatToolPathsAction> heat_client_;

  Ui::ToolPathParametersEditor* ui_;

  opp_msgs::ToolPath::Ptr tool_path_;

  shape_msgs::Mesh::Ptr mesh_;

  QProgressDialog* progress_dialog_;
};

}  // namespace opp_gui

#endif  // OPP_GUI_WIDGETS_TOOL_PATH_PARAMETERS_EDITOR_WIDGET_H
