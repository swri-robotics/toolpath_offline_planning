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
#ifndef OPP_MSGS_SERIALIZATION_TRAJECTORY_MSGS_YAML
#define OPP_MSGS_SERIALIZATION_TRAJECTORY_MSGS_YAML

#include <trajectory_msgs/JointTrajectory.h>
#include <opp_msgs_serialization/std_msgs_yaml.h>

namespace YAML
{

template<>
struct convert<trajectory_msgs::JointTrajectoryPoint>
{
  static Node encode(const trajectory_msgs::JointTrajectoryPoint& rhs)
  {
    Node node;
    node["positions"] = rhs.positions;
    node["velocities"] = rhs.velocities;
    node["accelerations"] = rhs.accelerations;
    node["effort"] = rhs.effort;
    node["time_from_start"] = rhs.time_from_start.toSec();

    return node;
  }

  static bool decode(const Node& node, trajectory_msgs::JointTrajectoryPoint& rhs)
  {
    if (node.size() != 5) return false;

    rhs.positions = node["positions"].as<std::vector<double> >();
    rhs.velocities = node["velocities"].as<std::vector<double> >();
    rhs.accelerations = node["accelerations"].as<std::vector<double> >();
    rhs.effort = node["effort"].as<std::vector<double> >();
    rhs.time_from_start = ros::Duration(node["time_from_start"].as<double>());

    return true;
  }
};

template<>
struct convert<trajectory_msgs::JointTrajectory>
{
  static Node encode(const trajectory_msgs::JointTrajectory& rhs)
  {
    Node node;
    node["header"] = rhs.header;
    node["joint_names"] = rhs.joint_names;
    node["points"] = rhs.points;
    return node;
  }

  static bool decode(const Node& node, trajectory_msgs::JointTrajectory& rhs)
  {
    if (node.size() != 3) return false;
  
    rhs.header = node["header"].as<std_msgs::Header>();
    rhs.joint_names = node["joint_names"].as<std::vector<std::string> >();
    rhs.points = node["points"].as<std::vector<trajectory_msgs::JointTrajectoryPoint> >(); 
  
    return true;
  }
};

}

#endif // OPP_MSGS_SERIALIZATION_TRAJECTORY_MSGS_YAML
