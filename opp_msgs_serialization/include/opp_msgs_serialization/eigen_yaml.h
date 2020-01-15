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
#ifndef OPP_MSGS_SERIALIZATION_EIGEN_YAML_H
#define OPP_MSGS_SERIALIZATION_EIGEN_YAML_H

#include <eigen_conversions/eigen_msg.h>

#include "opp_msgs_serialization/geometry_msgs_yaml.h"

namespace YAML
{
template <>
struct convert<Eigen::Affine3d>
{
  static Node encode(const Eigen::Affine3d& rhs)
  {
    geometry_msgs::Pose msg;
    tf::poseEigenToMsg(rhs, msg);
    Node node = Node(msg);
    return node;
  }

  static bool decode(const Node& node, Eigen::Affine3d& rhs)
  {
    geometry_msgs::Pose msg;
    msg = node.as<geometry_msgs::Pose>();
    tf::poseMsgToEigen(msg, rhs);
    return true;
  }
};

template <>
struct convert<Eigen::Vector3d>
{
  static Node encode(const Eigen::Vector3d& rhs)
  {
    geometry_msgs::Point msg;
    tf::pointEigenToMsg(rhs, msg);
    Node node = Node(msg);
    return node;
  }

  static bool decode(const Node& node, Eigen::Vector3d& rhs)
  {
    geometry_msgs::Point msg;
    msg = node.as<geometry_msgs::Point>();
    tf::pointMsgToEigen(msg, rhs);
    return true;
  }
};

}  // namespace YAML

#endif  // OPP_MSGS_SERIALIZATION_EIGEN_YAML_H
