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
#ifndef OPP_MSGS_SERIALIZATION_GEOMETRY_MSGS_YAML
#define OPP_MSGS_SERIALIZATION_GEOMETRY_MSGS_YAML

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include "opp_msgs_serialization/std_msgs_yaml.h"

namespace YAML
{

template<>
struct convert<geometry_msgs::Vector3>
{
  static Node encode(const geometry_msgs::Vector3& rhs)
  {
    Node node;
    node["x"] = rhs.x;
    node["y"] = rhs.y;
    node["z"] = rhs.z;

    return node;
  }

  static bool decode(const Node& node, geometry_msgs::Vector3& rhs)
  {
    if (node.size() != 3) return false;

    rhs.x = node["x"].as<double>();
    rhs.y = node["y"].as<double>();
    rhs.z = node["z"].as<double>();
    return true;
  }
};

template<>
struct convert<geometry_msgs::Point>
{
  static Node encode(const geometry_msgs::Point& rhs)
  {
    Node node;
    node["x"] = rhs.x;
    node["y"] = rhs.y;
    node["z"] = rhs.z;
    return node;
  }

  static bool decode(const Node& node, geometry_msgs::Point& rhs)
  {
    if (node.size() != 3) return false;

    rhs.x = node["x"].as<double>();
    rhs.y = node["y"].as<double>();
    rhs.z = node["z"].as<double>();
    
    return true;
  }
};


template<>
struct convert<geometry_msgs::Quaternion>
{
  static Node encode(const geometry_msgs::Quaternion& rhs)
  {
    Node node;
    node["x"] = rhs.x;
    node["y"] = rhs.y;
    node["z"] = rhs.z;
    node["w"] = rhs.w;

    return node;
  }

  static bool decode(const Node& node, geometry_msgs::Quaternion& rhs)
  {
    if (node.size() != 4) return false;

    rhs.x = node["x"].as<double>();
    rhs.y = node["y"].as<double>();
    rhs.z = node["z"].as<double>();
    rhs.w = node["w"].as<double>();

    return true;
  }
};

template<>
struct convert<geometry_msgs::Pose>
{
  static Node encode(const geometry_msgs::Pose& rhs)
  {
    Node node;
    node["position"] = rhs.position;
    node["orientation"] = rhs.orientation;

    return node;
  }

  static bool decode(const Node& node, geometry_msgs::Pose& rhs)
  {
    if (node.size() != 2) return false;

    rhs.position = node["position"].as<geometry_msgs::Point>();
    rhs.orientation = node["orientation"].as<geometry_msgs::Quaternion>();
    return true;
  }
};

template<>
struct convert<geometry_msgs::PoseStamped>
{
  static Node encode(const geometry_msgs::PoseStamped& rhs)
  {
    Node node;
    node["header"] = rhs.header;
    node["pose"] = rhs.pose;

    return node;
  }

  static bool decode(const Node& node, geometry_msgs::PoseStamped& rhs)
  {
    if (node.size() != 2) return false;

    rhs.header = node["header"].as<std_msgs::Header>();
    rhs.pose = node["pose"].as<geometry_msgs::Pose>();
    return true;
  }
};

template<>
struct convert<geometry_msgs::PoseArray>
{
  static Node encode(const geometry_msgs::PoseArray& rhs)
  {
    Node node;
    node["header"] = rhs.header;
    node["poses"] = rhs.poses;

    return node;
  }

  static bool decode(const Node& node, geometry_msgs::PoseArray& rhs)
  {
    if (node.size() != 2) return false;

    rhs.header = node["header"].as<std_msgs::Header>();
    rhs.poses = node["poses"].as<std::vector<geometry_msgs::Pose> >();
    return true;
  }
};

template<>
struct convert<geometry_msgs::Transform>
{
  static Node encode(const geometry_msgs::Transform& rhs)
  {
    Node node;
    node["rotation"] = rhs.rotation;
    node["translation"] = rhs.translation;

    return node;
  }

  static bool decode(const Node& node, geometry_msgs::Transform& rhs)
  {
    if (node.size() != 2) return false;

    rhs.rotation = node["rotation"].as<geometry_msgs::Quaternion>();
    rhs.translation = node["translation"].as<geometry_msgs::Vector3>();

    return true;
  }
};

template<>
struct convert<geometry_msgs::TransformStamped>
{
  static Node encode(const geometry_msgs::TransformStamped& rhs)
  {
    Node node;
    node["header"] = rhs.header;
    node["child_frame_id"] = rhs.child_frame_id;
    node["transform"] = rhs.transform;

    return node;
  }

  static bool decode(const Node& node, geometry_msgs::TransformStamped& rhs)
  {
    if (node.size() != 3) return false;

    rhs.header = node["header"].as<std_msgs::Header>();
    rhs.child_frame_id = node["child_frame_id"].as<std::string>();
    rhs.transform = node["transform"].as<geometry_msgs::Transform>();
    return true;
  }
};

}

#endif // OPP_MSGS_SERIALIZATION_GEOMETRY_MSGS_YAML
