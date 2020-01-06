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
#ifndef OPP_MSGS_SERIALIZATION_SENSOR_MSGS_YAML
#define OPP_MSGS_SERIALIZATION_SENSOR_MSGS_YAML

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>

#include "opp_msgs_serialization/std_msgs_yaml.h"

namespace YAML
{

template<>
struct convert<sensor_msgs::RegionOfInterest>
{
  static Node encode(const sensor_msgs::RegionOfInterest& rhs)
  {
    Node node;

    node["x_offset"] = rhs.x_offset;
    node["y_offset"] = rhs.y_offset;
    node["height"] = rhs.height;
    node["width"] = rhs.width;
    node["do_rectify"] = rhs.do_rectify;

    return node;
  }

  static bool decode(const Node& node, sensor_msgs::RegionOfInterest& rhs)
  {
    if (node.size() != 5) return false;

    rhs.x_offset = node["x_offset"].as<decltype (rhs.x_offset)>();
    rhs.y_offset = node["y_offset"].as<decltype (rhs.y_offset)>();
    rhs.height = node["height"].as<decltype (rhs.height)>();
    rhs.width = node["width"].as<decltype (rhs.width)>();
    rhs.do_rectify = node["do_rectify"].as<decltype (rhs.do_rectify)>() ;

    return true;
  }
};

template<>
struct convert<sensor_msgs::CameraInfo>
{
  static Node encode(const sensor_msgs::CameraInfo& rhs)
  {
    Node node;

    node["header"] = rhs.header;
    node["height"] = rhs.height;
    node["width"] = rhs.width;
    node["distortion_model"] = rhs.distortion_model;

    std::vector<double> K_vec, R_vec, P_vec;

    std::copy_n(rhs.K.begin(), rhs.K.size(), std::back_inserter(K_vec));
    std::copy_n(rhs.R.begin(), rhs.R.size(), std::back_inserter(R_vec));
    std::copy_n(rhs.P.begin(), rhs.P.size(), std::back_inserter(P_vec));

    node["D"] = rhs.D;
    node["K"] = K_vec;
    node["R"] = R_vec;
    node["P"] = P_vec;
    node["binning_x"] = rhs.binning_x;
    node["binning_y"] = rhs.binning_y;
    node["roi"] = rhs.roi;

    return node;
  }

  static bool decode(const Node& node, sensor_msgs::CameraInfo& rhs)
  {
    if (node.size() != 11) return false;

    rhs.header = node["header"].as<decltype(rhs.header)>();
    rhs.height = node["height"].as<decltype(rhs.height)>();
    rhs.width = node["width"].as<decltype(rhs.width)>();
    rhs.distortion_model = node["distortion_model"].as<decltype(rhs.distortion_model)>();
    rhs.D = node["D"].as<decltype(rhs.D)>();

    std::vector<double> K_vec, R_vec, P_vec;
    K_vec = node["K"].as<decltype(K_vec)>();
    R_vec = node["R"].as<decltype(R_vec)>();
    P_vec = node["P"].as<decltype(P_vec)>();

    std::copy_n(K_vec.begin(), K_vec.size(), rhs.K.begin());
    std::copy_n(R_vec.begin(), R_vec.size(), rhs.R.begin());
    std::copy_n(P_vec.begin(), P_vec.size(), rhs.P.begin());

    rhs.binning_x = node["binning_x"].as<decltype(rhs.binning_x)>();
    rhs.binning_y = node["binning_y"].as<decltype(rhs.binning_y)>();
    rhs.roi = node["roi"].as<decltype(rhs.roi)>();

    return true;
  }
};

template<>
struct convert<sensor_msgs::JointState>
{
  static Node encode(const sensor_msgs::JointState& rhs)
  {
    Node node;

    node["header"] = rhs.header;
    node["name"] = rhs.name;
    node["position"] = rhs.position;
    node["velocity"] = rhs.velocity;
    node["effort"] = rhs.effort;

    return node;
  }

  static bool decode(const Node& node, sensor_msgs::JointState& rhs)
  {
    if (node.size() != 5) return false;

    rhs.header = node["header"].as<decltype (rhs.header)>();
    rhs.name = node["name"].as<decltype (rhs.name)>();
    rhs.position = node["position"].as<decltype (rhs.position)>();
    rhs.velocity = node["velocity"].as<decltype (rhs.velocity)>();
    rhs.effort = node["effort"].as<decltype (rhs.effort)>() ;

    return true;
  }
};

} // namespace YAML

#endif // OPP_MSGS_SERIALIZATION_SENSOR_MSGS_YAML
