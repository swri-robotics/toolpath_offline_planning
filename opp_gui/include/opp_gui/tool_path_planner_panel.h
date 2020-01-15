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

#ifndef OPP_GUI_TOOL_PATH_PLANNER_PANEL_H
#define OPP_GUI_TOOL_PATH_PLANNER_PANEL_H

#include <ros/node_handle.h>
#include <rviz/panel.h>

namespace opp_gui
{
class ToolPathPlannerWidget;

/**
 * @brief Simple RViz panel that wraps the tool path planner widget such that can easily
 * be used within the context of RViz
 */
class ToolPathPlannerPanel : public rviz::Panel
{
  Q_OBJECT
public:
  /**
   * @brief ToolPathPlannerPanel
   *
   * Note: the visualization manager object, inherited from `rviz::Panel`, cannot be used in
   * the constructor. Even though it is a valid, non-null object, it will not be initialized
   * until the constructor has finished.
   * @param parent
   */
  ToolPathPlannerPanel(QWidget* parent = nullptr);

  /**
   * @brief Initializes the TPP widget and uses the visualization manager to gather
   * information about the available frames (used to visualize markers).
   */
  virtual void onInitialize() override;

private:
  ToolPathPlannerWidget* tpp_widget_;

  ros::NodeHandle nh_;
};

}  // namespace opp_gui

#endif  // OPP_GUI_TOOL_PATH_PLANNER_PANEL_H
