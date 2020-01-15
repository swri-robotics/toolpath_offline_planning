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

#include "opp_gui/widgets/touch_point_editor_widget.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "touch_point_editor_demo_app");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string fixed_frame;
  pnh.param<std::string>("fixed_frame", fixed_frame, "map");
  ROS_INFO_STREAM("Using fixed frame '" << fixed_frame << "' for displays");

  QApplication app(argc, argv);

  opp_gui::TouchPointEditorWidget* touch_point_editor_widget =
      new opp_gui::TouchPointEditorWidget(nullptr, nh, { fixed_frame });
  touch_point_editor_widget->show();

  app.exec();

  std::map<std::string, opp_msgs::TouchPoint> tps = touch_point_editor_widget->getPoints();
  for (const std::pair<const std::string, opp_msgs::TouchPoint>& pair : tps)
  {
    ROS_INFO_STREAM("Touch Point: " << pair.first << "\nInfo:\n" << pair.second);
  }

  delete touch_point_editor_widget;

  return 0;
}
