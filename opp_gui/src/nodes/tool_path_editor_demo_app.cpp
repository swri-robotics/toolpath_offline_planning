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
#include "opp_gui/widgets/tool_path_editor_widget.h"

template <typename T>
bool get(const ros::NodeHandle& nh, const std::string& key, T& val)
{
  if (!nh.getParam(key, val))
  {
    ROS_ERROR_STREAM("Failed to get '" << key << "' parameter");
    return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "touch_point_editor_demo_app");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string mesh_resource;
  if (!get(pnh, "mesh_resource", mesh_resource))
    return -1;

  double intersecting_plane_height;
  pnh.param<double>("intersecting_plane_height", intersecting_plane_height, 0.1);

  std::string fixed_frame;
  pnh.param<std::string>("fixed_frame", fixed_frame, "map");
  ROS_INFO_STREAM("Using fixed frame '" << fixed_frame << "' for displays");

  // Create the mesh message from the input resource
  shape_msgs::Mesh mesh_msg;
  if (!opp_gui::utils::getMeshMsgFromResource(mesh_resource, mesh_msg))
    return -1;

  // Create and start the Qt application
  QApplication app(argc, argv);

  opp_gui::ToolPathEditorWidget* tool_path_editor_widget =
      new opp_gui::ToolPathEditorWidget(nullptr, nh, fixed_frame, fixed_frame, fixed_frame);
  tool_path_editor_widget->init(mesh_msg);
  tool_path_editor_widget->show();

  app.exec();

  // Get the tool path data after the application is done running
  opp_gui::ToolPathDataMap data = tool_path_editor_widget->getToolPathData();
  for (const std::pair<const std::string, opp_msgs::ToolPath>& pair : data)
  {
    ROS_INFO_STREAM("Tool Path: " << pair.first << "\nTool Path (including config):\n" << pair.second);
  }

  delete tool_path_editor_widget;

  return 0;
}
