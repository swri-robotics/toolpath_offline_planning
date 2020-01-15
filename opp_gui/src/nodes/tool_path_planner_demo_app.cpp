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

#include <QApplication>

#include <ros/ros.h>

#include "opp_gui/utils.h"
#include "opp_gui/widgets/tool_path_planner_widget.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tool_path_planner_demo_app");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string fixed_frame;
  pnh.param<std::string>("fixed_frame", fixed_frame, "map");
  ROS_INFO_STREAM("Using fixed frame '" << fixed_frame << "' for displays");

  // Create and start the Qt application
  QApplication app(argc, argv);

  opp_gui::ToolPathPlannerWidget* tpp_widget = new opp_gui::ToolPathPlannerWidget(nullptr, nh, { fixed_frame });
  tpp_widget->show();

  app.exec();

  delete tpp_widget;

  return 0;
}
