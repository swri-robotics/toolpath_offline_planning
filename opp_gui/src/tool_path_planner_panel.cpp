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

#include "opp_gui/tool_path_planner_panel.h"

#include <QVBoxLayout>

#include <rviz/display.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include "opp_gui/widgets/tool_path_planner_widget.h"

namespace opp_gui
{
ToolPathPlannerPanel::ToolPathPlannerPanel(QWidget* parent) : rviz::Panel(parent) {}

void ToolPathPlannerPanel::onInitialize()
{
  // Add the current fixed frame to the list if it doesn't exist in the frames that TF knows about
  std::vector<std::string> frames;
  std::string fixed_frame = vis_manager_->getFixedFrame().toStdString();
  frames.insert(frames.begin(), fixed_frame);

  tpp_widget_ = new ToolPathPlannerWidget(this, nh_, frames);

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(tpp_widget_);

  setLayout(layout);
}

}  // namespace opp_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opp_gui::ToolPathPlannerPanel, rviz::Panel)
