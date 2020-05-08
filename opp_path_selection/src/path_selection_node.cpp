/*
 * Copyright 2018 Southwest Research Institute
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

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "opp_path_selection/path_selector.h"
#include "opp_path_selection/path_selection_artist.h"

static const float TIMEOUT = 15.0;

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "roi_selection_node");

  // Set up ROS node handle
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Get ROS parameters
  std::string world_frame;
  if (!pnh.getParam("world_frame", world_frame))
  {
    ROS_FATAL("'world_frame' parameter must be set");
    return 1;
  }

  std::string sensor_data_frame;
  if (!pnh.getParam("sensor_data_frame", sensor_data_frame))
  {
    ROS_FATAL("'sensor_data_frame' parameter must be set");
    return 1;
  }

  // Wait for tf to initialize
  tf::TransformListener listener;
  if (!listener.waitForTransform(
          world_frame, sensor_data_frame, ros::Time(0), ros::Duration(static_cast<double>(TIMEOUT))))
  {
    ROS_ERROR("Transform lookup between '%s' and '%s' timed out", world_frame.c_str(), sensor_data_frame.c_str());
    return -1;
  }

  // Set up the selection artist
  opp_path_selection::PathSelectionArtist artist(nh, world_frame, sensor_data_frame);

  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
